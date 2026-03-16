#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cctype>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <sys/wait.h>
#include <unistd.h>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace mr2_video_streaming {

using json = nlohmann::json;

enum class StreamSourceType { RosTopic, V4L2 };

struct EncoderConfig {
  int bitrate_kbps{1500};
  int keyframe_interval{30};
  std::string speed_preset{"ultrafast"};
  std::string tune{"zerolatency"};
};

struct StreamConfig {
  std::string stream_id;
  StreamSourceType source_type{StreamSourceType::RosTopic};
  std::string ros_topic;
  std::string ros_encoding;
  std::string v4l2_device;
  std::string v4l2_pixel_format;
  int udp_port{0};
  int width{0};
  int height{0};
  int framerate{15};
  EncoderConfig encoder;
};

std::string uppercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::toupper(ch));
  });
  return value;
}

bool is_supported_ros_encoding(const std::string & encoding) {
  return encoding == "rgb8" || encoding == "bgr8";
}

std::string source_type_to_string(StreamSourceType source_type) {
  switch (source_type) {
    case StreamSourceType::RosTopic:
      return "ros_topic";
    case StreamSourceType::V4L2:
      return "v4l2";
  }
  return "unknown";
}

std::string quote_gstreamer_string(const std::string & value) {
  std::string quoted = "\"";
  for (const char ch : value) {
    if (ch == '\\' || ch == '"') {
      quoted.push_back('\\');
    }
    quoted.push_back(ch);
  }
  quoted.push_back('"');
  return quoted;
}

class StreamPipeline {
 public:
  StreamPipeline(StreamConfig config, std::string base_host, rclcpp::Logger logger)
      : config_(std::move(config)),
        base_host_(std::move(base_host)),
        logger_(std::move(logger)),
        frame_duration_ns_(1000000000LL / std::max(1, config_.framerate)) {}

  ~StreamPipeline() { shutdown(); }

  void start_if_needed() {
    if (config_.source_type != StreamSourceType::V4L2 || pipeline_ != nullptr || gst_child_pid_ > 0) {
      return;
    }
    start_v4l2_pipeline();
  }

  void poll_runtime() {
    if (config_.source_type != StreamSourceType::V4L2 || stopping_) {
      return;
    }

    if (gst_child_pid_ > 0) {
      int status = 0;
      const pid_t result = waitpid(gst_child_pid_, &status, WNOHANG);
      if (result == 0) {
        return;
      }

      if (result == gst_child_pid_) {
        if (WIFEXITED(status)) {
          RCLCPP_WARN(
              logger_,
              "V4L2 sender child for stream %s exited with code %d; scheduling restart",
              config_.stream_id.c_str(),
              WEXITSTATUS(status));
        } else if (WIFSIGNALED(status)) {
          RCLCPP_WARN(
              logger_,
              "V4L2 sender child for stream %s exited on signal %d; scheduling restart",
              config_.stream_id.c_str(),
              WTERMSIG(status));
        } else {
          RCLCPP_WARN(
              logger_,
              "V4L2 sender child for stream %s exited unexpectedly; scheduling restart",
              config_.stream_id.c_str());
        }
        gst_child_pid_ = -1;
        next_v4l2_restart_at_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
      } else if (result < 0) {
        if (errno != ECHILD) {
          RCLCPP_ERROR(
              logger_,
              "waitpid failed for V4L2 sender child on stream %s: %s",
              config_.stream_id.c_str(),
              std::strerror(errno));
          return;
        }

        gst_child_pid_ = -1;
        next_v4l2_restart_at_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
      }
    }

    if (gst_child_pid_ <= 0 && std::chrono::steady_clock::now() >= next_v4l2_restart_at_) {
      try {
        start_v4l2_pipeline();
      } catch (const std::exception & error) {
        next_v4l2_restart_at_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        RCLCPP_ERROR(
            logger_,
            "Failed to restart V4L2 sender for stream %s: %s",
            config_.stream_id.c_str(),
            error.what());
      }
    }
  }

  void push_frame(const sensor_msgs::msg::Image & msg) {
    if (config_.source_type != StreamSourceType::RosTopic) {
      return;
    }

    if (!validate_encoding(msg.encoding)) {
      return;
    }

    if (!ensure_ros_pipeline(static_cast<int>(msg.width), static_cast<int>(msg.height))) {
      return;
    }

    std::vector<uint8_t> normalized;
    if (!normalize_to_rgb(msg, normalized)) {
      return;
    }

    GstBuffer * buffer = gst_buffer_new_allocate(nullptr, normalized.size(), nullptr);
    if (buffer == nullptr) {
      RCLCPP_ERROR(logger_, "Failed to allocate GstBuffer for stream %s", config_.stream_id.c_str());
      return;
    }

    gst_buffer_fill(buffer, 0, normalized.data(), normalized.size());
    GST_BUFFER_PTS(buffer) = frame_index_ * frame_duration_ns_;
    GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);
    GST_BUFFER_DURATION(buffer) = frame_duration_ns_;
    frame_index_ += 1;

    const GstFlowReturn result = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
    if (result != GST_FLOW_OK) {
      RCLCPP_WARN(
          logger_,
          "Dropped frame for stream %s because appsrc returned flow status %d",
          config_.stream_id.c_str(),
          static_cast<int>(result));
    }
  }

 private:
  bool validate_encoding(const std::string & encoding) {
    if (encoding != config_.ros_encoding) {
      RCLCPP_WARN(
          logger_,
          "Stream %s expected encoding %s but received %s",
          config_.stream_id.c_str(),
          config_.ros_encoding.c_str(),
          encoding.c_str());
      return false;
    }

    if (!is_supported_ros_encoding(encoding)) {
      RCLCPP_WARN(
          logger_,
          "Stream %s received unsupported encoding %s",
          config_.stream_id.c_str(),
          encoding.c_str());
      return false;
    }

    return true;
  }

  std::string build_encoded_sink_branch() const {
    std::ostringstream branch;
    branch << "! x264enc bitrate=" << config_.encoder.bitrate_kbps
           << " speed-preset=" << config_.encoder.speed_preset
           << " tune=" << config_.encoder.tune
           << " key-int-max=" << config_.encoder.keyframe_interval
           << " bframes=0 byte-stream=true threads=1 "
           << "! h264parse config-interval=1 "
           << "! rtph264pay pt=96 mtu=1200 config-interval=1 "
           << "! udpsink host=" << quote_gstreamer_string(base_host_)
           << " port=" << config_.udp_port
           << " sync=false async=false";
    return branch.str();
  }

  std::string build_raw_output_caps() const {
    std::ostringstream caps;
    caps << "video/x-raw,format=I420";
    if (width_ > 0) {
      caps << ",width=" << width_;
    }
    if (height_ > 0) {
      caps << ",height=" << height_;
    }
    if (config_.framerate > 0) {
      caps << ",framerate=" << std::max(1, config_.framerate) << "/1";
    }
    return caps.str();
  }

  std::string build_v4l2_source_caps() const {
    std::ostringstream caps;
    const std::string pixel_format = uppercase(config_.v4l2_pixel_format);
    const bool is_jpeg = pixel_format == "MJPG" || pixel_format == "JPEG";

    caps << (is_jpeg ? "image/jpeg" : "video/x-raw");
    if (!is_jpeg && !pixel_format.empty()) {
      caps << ",format=" << pixel_format;
    }
    if (config_.width > 0) {
      caps << ",width=" << config_.width;
    }
    if (config_.height > 0) {
      caps << ",height=" << config_.height;
    }
    if (config_.framerate > 0) {
      caps << ",framerate=" << std::max(1, config_.framerate) << "/1";
    }
    return caps.str();
  }

  bool ensure_ros_pipeline(int frame_width, int frame_height) {
    if (pipeline_ != nullptr) {
      if (frame_width != width_ || frame_height != height_) {
        RCLCPP_WARN(
            logger_,
            "Stream %s received unexpected frame dimensions %dx%d (expected %dx%d)",
            config_.stream_id.c_str(),
            frame_width,
            frame_height,
            width_,
            height_);
        return false;
      }
      return true;
    }

    width_ = config_.width > 0 ? config_.width : frame_width;
    height_ = config_.height > 0 ? config_.height : frame_height;

    if (width_ != frame_width || height_ != frame_height) {
      RCLCPP_WARN(
          logger_,
          "Stream %s configured for %dx%d but first frame is %dx%d; using configured size and dropping frame",
          config_.stream_id.c_str(),
          width_,
          height_,
          frame_width,
          frame_height);
      return false;
    }

    std::ostringstream pipeline_description;
    pipeline_description
        << "appsrc name=src is-live=true format=time block=false do-timestamp=false max-buffers=1 "
        << "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
        << "! videoconvert "
        << "! " << build_raw_output_caps() << " "
        << build_encoded_sink_branch();

    start_pipeline(pipeline_description.str(), true);

    RCLCPP_INFO(
        logger_,
        "Started ROS video stream %s topic=%s port=%d size=%dx%d fps=%d",
        config_.stream_id.c_str(),
        config_.ros_topic.c_str(),
        config_.udp_port,
        width_,
        height_,
        config_.framerate);
    return true;
  }

  void start_v4l2_pipeline() {
    if (gst_child_pid_ > 0) {
      return;
    }

    width_ = config_.width;
    height_ = config_.height;

    const std::string pixel_format = uppercase(config_.v4l2_pixel_format);
    const bool is_jpeg = pixel_format == "MJPG" || pixel_format == "JPEG";

    std::vector<std::string> args{
        "gst-launch-1.0",
        "-q",
        "v4l2src",
        "device=" + config_.v4l2_device,
    };
    const std::string source_caps = build_v4l2_source_caps();
    if (!source_caps.empty()) {
      args.push_back("!");
      args.push_back(source_caps);
    }
    if (is_jpeg) {
      args.push_back("!");
      args.push_back("jpegdec");
    }
    args.insert(
        args.end(),
        {
            "!",
            "videoconvert",
            "!",
            "video/x-raw,format=I420",
            "!",
            "x264enc",
            "bitrate=" + std::to_string(config_.encoder.bitrate_kbps),
            "speed-preset=" + config_.encoder.speed_preset,
            "tune=" + config_.encoder.tune,
            "key-int-max=" + std::to_string(config_.encoder.keyframe_interval),
            "bframes=0",
            "byte-stream=true",
            "threads=1",
            "!",
            "h264parse",
            "config-interval=1",
            "!",
            "rtph264pay",
            "pt=96",
            "mtu=1200",
            "config-interval=1",
            "!",
            "udpsink",
            "host=" + base_host_,
            "port=" + std::to_string(config_.udp_port),
            "sync=false",
            "async=false",
        });

    start_gst_launch_child(args);
    next_v4l2_restart_at_ = std::chrono::steady_clock::time_point::max();

    std::ostringstream details;
    details << "Started V4L2 video stream " << config_.stream_id
            << " device=" << config_.v4l2_device
            << " port=" << config_.udp_port;
    if (config_.width > 0 && config_.height > 0) {
      details << " size=" << config_.width << "x" << config_.height;
    }
    if (config_.framerate > 0) {
      details << " fps=" << config_.framerate;
    }
    if (!config_.v4l2_pixel_format.empty()) {
      details << " pixel_format=" << config_.v4l2_pixel_format;
    }
    RCLCPP_INFO(logger_, "%s", details.str().c_str());
  }

  void start_pipeline(const std::string & description, bool needs_appsrc) {
    GError * error = nullptr;
    pipeline_ = gst_parse_launch(description.c_str(), &error);
    if (pipeline_ == nullptr) {
      const std::string message = error != nullptr ? error->message : "unknown error";
      if (error != nullptr) {
        g_error_free(error);
      }
      throw std::runtime_error("failed to create pipeline for " + config_.stream_id + ": " + message);
    }

    if (needs_appsrc) {
      appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
      if (appsrc_ == nullptr) {
        throw std::runtime_error("failed to resolve appsrc for stream " + config_.stream_id);
      }

      GstCaps * caps = gst_caps_new_simple(
          "video/x-raw",
          "format",
          G_TYPE_STRING,
          "RGB",
          "width",
          G_TYPE_INT,
          width_,
          "height",
          G_TYPE_INT,
          height_,
          "framerate",
          GST_TYPE_FRACTION,
          std::max(1, config_.framerate),
          1,
          nullptr);
      g_object_set(
          G_OBJECT(appsrc_),
          "caps",
          caps,
          "is-live",
          TRUE,
          "format",
          GST_FORMAT_TIME,
          "block",
          FALSE,
          "do-timestamp",
          FALSE,
          "max-buffers",
          1,
          nullptr);
      gst_caps_unref(caps);
    }

    bus_ = gst_element_get_bus(pipeline_);
    if (bus_ != nullptr) {
      bus_watch_id_ = gst_bus_add_watch(bus_, &StreamPipeline::handle_bus_message, this);
    }

    const GstStateChangeReturn state_change = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state_change == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("failed to start pipeline for stream " + config_.stream_id);
    }
    if (state_change == GST_STATE_CHANGE_ASYNC) {
      GstState current_state = GST_STATE_NULL;
      const GstStateChangeReturn settled =
          gst_element_get_state(pipeline_, &current_state, nullptr, 5 * GST_SECOND);
      if (settled == GST_STATE_CHANGE_FAILURE) {
        throw std::runtime_error("pipeline failed while settling to PLAYING for stream " + config_.stream_id);
      }
    }
  }

  bool normalize_to_rgb(const sensor_msgs::msg::Image & msg, std::vector<uint8_t> & output) {
    const std::size_t expected_row_bytes = static_cast<std::size_t>(msg.width) * 3U;
    if (msg.step < expected_row_bytes) {
      RCLCPP_WARN(
          logger_,
          "Stream %s received invalid image step %u for width %u",
          config_.stream_id.c_str(),
          msg.step,
          msg.width);
      return false;
    }

    output.resize(static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height) * 3U);
    for (std::size_t row = 0; row < msg.height; ++row) {
      const uint8_t * src = msg.data.data() + row * msg.step;
      uint8_t * dst = output.data() + row * expected_row_bytes;
      if (msg.encoding == "rgb8") {
        std::copy(src, src + expected_row_bytes, dst);
        continue;
      }

      for (std::size_t column = 0; column < msg.width; ++column) {
        const std::size_t src_offset = column * 3U;
        const std::size_t dst_offset = column * 3U;
        dst[dst_offset + 0] = src[src_offset + 2];
        dst[dst_offset + 1] = src[src_offset + 1];
        dst[dst_offset + 2] = src[src_offset + 0];
      }
    }

    return true;
  }

  void shutdown() {
    stopping_ = true;
    if (gst_child_pid_ > 0) {
      kill(gst_child_pid_, SIGTERM);
      waitpid(gst_child_pid_, nullptr, 0);
      gst_child_pid_ = -1;
    }
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (bus_watch_id_ != 0) {
      g_source_remove(bus_watch_id_);
      bus_watch_id_ = 0;
    }
    if (bus_ != nullptr) {
      gst_object_unref(bus_);
      bus_ = nullptr;
    }
    if (appsrc_ != nullptr) {
      gst_object_unref(appsrc_);
      appsrc_ = nullptr;
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

  StreamConfig config_;
  std::string base_host_;
  rclcpp::Logger logger_;
  GstElement * pipeline_{nullptr};
  GstElement * appsrc_{nullptr};
  GstBus * bus_{nullptr};
  guint bus_watch_id_{0};
  pid_t gst_child_pid_{-1};
  int width_{0};
  int height_{0};
  GstClockTime frame_duration_ns_{0};
  GstClockTime frame_index_{0};
  bool stopping_{false};
  std::chrono::steady_clock::time_point next_v4l2_restart_at_{
      std::chrono::steady_clock::time_point::max()};

  static gboolean handle_bus_message(GstBus * /* bus */, GstMessage * message, gpointer user_data) {
    auto * self = static_cast<StreamPipeline *>(user_data);
    return self->on_bus_message(message);
  }

  gboolean on_bus_message(GstMessage * message) {
    switch (GST_MESSAGE_TYPE(message)) {
      case GST_MESSAGE_ERROR: {
        GError * error = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_error(message, &error, &debug);
        RCLCPP_ERROR(
            logger_,
            "Stream %s GStreamer error from %s: %s%s%s",
            config_.stream_id.c_str(),
            GST_OBJECT_NAME(message->src),
            error != nullptr ? error->message : "unknown error",
            debug != nullptr ? " | " : "",
            debug != nullptr ? debug : "");
        if (error != nullptr) {
          g_error_free(error);
        }
        if (debug != nullptr) {
          g_free(debug);
        }
        break;
      }
      case GST_MESSAGE_WARNING: {
        GError * warning = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_warning(message, &warning, &debug);
        RCLCPP_WARN(
            logger_,
            "Stream %s GStreamer warning from %s: %s%s%s",
            config_.stream_id.c_str(),
            GST_OBJECT_NAME(message->src),
            warning != nullptr ? warning->message : "unknown warning",
            debug != nullptr ? " | " : "",
            debug != nullptr ? debug : "");
        if (warning != nullptr) {
          g_error_free(warning);
        }
        if (debug != nullptr) {
          g_free(debug);
        }
        break;
      }
      case GST_MESSAGE_STATE_CHANGED: {
        if (GST_MESSAGE_SRC(message) == GST_OBJECT(pipeline_)) {
          GstState old_state = GST_STATE_NULL;
          GstState new_state = GST_STATE_NULL;
          GstState pending_state = GST_STATE_NULL;
          gst_message_parse_state_changed(message, &old_state, &new_state, &pending_state);
          RCLCPP_INFO(
              logger_,
              "Stream %s pipeline state %s -> %s (pending %s)",
              config_.stream_id.c_str(),
              gst_element_state_get_name(old_state),
              gst_element_state_get_name(new_state),
              gst_element_state_get_name(pending_state));
        }
        break;
      }
      default:
        break;
    }
    return G_SOURCE_CONTINUE;
  }

  void start_gst_launch_child(const std::vector<std::string> & args) {
    std::vector<char *> argv;
    argv.reserve(args.size() + 1);
    for (const auto & arg : args) {
      argv.push_back(const_cast<char *>(arg.c_str()));
    }
    argv.push_back(nullptr);

    gst_child_pid_ = fork();
    if (gst_child_pid_ < 0) {
      throw std::runtime_error(
          "failed to fork gst-launch child for stream " + config_.stream_id + ": " +
          std::strerror(errno));
    }

    if (gst_child_pid_ == 0) {
      execvp(argv[0], argv.data());
      std::fprintf(
          stderr,
          "video_streaming_node failed to exec %s for stream %s: %s\n",
          argv[0],
          config_.stream_id.c_str(),
          std::strerror(errno));
      _exit(127);
    }
  }
};

StreamConfig parse_stream_config(const json & item) {
  StreamConfig config;
  config.stream_id = item.at("stream_id").get<std::string>();
  config.udp_port = item.at("udp_port").get<int>();
  config.width = item.value("width", 0);
  config.height = item.value("height", 0);
  config.framerate = std::max(1, item.value("framerate", 15));

  const json encoder = item.value("encoder", json::object());
  config.encoder.bitrate_kbps = std::max(100, encoder.value("bitrate_kbps", 1500));
  config.encoder.keyframe_interval = std::max(1, encoder.value("keyframe_interval", 30));
  config.encoder.speed_preset = encoder.value("speed_preset", std::string("ultrafast"));
  config.encoder.tune = encoder.value("tune", std::string("zerolatency"));

  if (item.contains("source")) {
    const json source = item.at("source");
    const std::string source_type = source.at("type").get<std::string>();
    if (source_type == "ros_topic") {
      config.source_type = StreamSourceType::RosTopic;
      config.ros_topic = source.at("ros_topic").get<std::string>();
      config.ros_encoding = source.at("ros_encoding").get<std::string>();
    } else if (source_type == "v4l2") {
      config.source_type = StreamSourceType::V4L2;
      config.v4l2_device = source.at("device").get<std::string>();
      config.v4l2_pixel_format = source.value("pixel_format", std::string());
    } else {
      throw std::runtime_error(
          "stream " + config.stream_id + " has unsupported source.type " + source_type);
    }
  } else {
    config.source_type = StreamSourceType::RosTopic;
    config.ros_topic = item.at("ros_topic").get<std::string>();
    config.ros_encoding = item.at("ros_encoding").get<std::string>();
  }

  if (config.source_type == StreamSourceType::RosTopic) {
    if (!is_supported_ros_encoding(config.ros_encoding)) {
      throw std::runtime_error(
          "stream " + config.stream_id + " has unsupported ros_encoding " + config.ros_encoding);
    }
  } else if (config.v4l2_device.empty()) {
    throw std::runtime_error("stream " + config.stream_id + " is missing source.device");
  }

  return config;
}

std::vector<StreamConfig> load_streams_from_file(const std::string & config_path) {
  std::ifstream input(config_path);
  if (!input.is_open()) {
    throw std::runtime_error("failed to open video config: " + config_path);
  }

  const json parsed = json::parse(input);
  const auto streams_json = parsed.at("streams");
  if (!streams_json.is_array() || streams_json.empty()) {
    throw std::runtime_error("video config must contain a non-empty streams array");
  }

  std::vector<StreamConfig> streams;
  streams.reserve(streams_json.size());
  std::set<std::string> stream_ids;
  std::set<std::string> ros_topics;
  std::set<std::string> v4l2_devices;
  std::set<int> udp_ports;

  for (const auto & item : streams_json) {
    StreamConfig stream = parse_stream_config(item);
    if (!stream_ids.insert(stream.stream_id).second) {
      throw std::runtime_error("duplicate stream_id " + stream.stream_id);
    }
    if (!udp_ports.insert(stream.udp_port).second) {
      throw std::runtime_error("duplicate udp_port " + std::to_string(stream.udp_port));
    }
    if (stream.source_type == StreamSourceType::RosTopic) {
      if (!ros_topics.insert(stream.ros_topic).second) {
        throw std::runtime_error("duplicate ros_topic " + stream.ros_topic);
      }
    } else if (!v4l2_devices.insert(stream.v4l2_device).second) {
      throw std::runtime_error("duplicate v4l2 device " + stream.v4l2_device);
    }
    streams.push_back(std::move(stream));
  }
  return streams;
}

class VideoStreamingNode : public rclcpp::Node {
 public:
  VideoStreamingNode()
      : rclcpp::Node("video_streaming"),
        video_config_path_(declare_parameter<std::string>("video_config_path", "")),
        base_host_(declare_parameter<std::string>("base_host", "127.0.0.1")),
        gst_main_loop_(g_main_loop_new(nullptr, FALSE)) {
    if (video_config_path_.empty()) {
      throw std::runtime_error("video_config_path parameter is required");
    }

    gst_main_loop_thread_ = std::thread([this]() {
      g_main_loop_run(gst_main_loop_);
    });

    const auto streams = load_streams_from_file(video_config_path_);
    for (const auto & stream : streams) {
      auto pipeline = std::make_shared<StreamPipeline>(stream, base_host_, get_logger());

      if (stream.source_type == StreamSourceType::RosTopic) {
        auto subscription = create_subscription<sensor_msgs::msg::Image>(
            stream.ros_topic,
            rclcpp::SensorDataQoS(),
            [pipeline](const sensor_msgs::msg::Image::SharedPtr msg) {
              pipeline->push_frame(*msg);
            });
        subscriptions_.push_back(subscription);
      } else {
        pipeline->start_if_needed();
      }

      pipelines_.push_back(pipeline);

      std::ostringstream details;
      details << "Configured video stream " << stream.stream_id
              << " source=" << source_type_to_string(stream.source_type)
              << " port=" << stream.udp_port;
      if (stream.source_type == StreamSourceType::RosTopic) {
        details << " topic=" << stream.ros_topic
                << " encoding=" << stream.ros_encoding;
      } else {
        details << " device=" << stream.v4l2_device;
        if (!stream.v4l2_pixel_format.empty()) {
          details << " pixel_format=" << stream.v4l2_pixel_format;
        }
      }
      RCLCPP_INFO(get_logger(), "%s", details.str().c_str());
    }

    monitor_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
      for (const auto & pipeline : pipelines_) {
        pipeline->poll_runtime();
      }
    });
  }

  ~VideoStreamingNode() override {
    if (gst_main_loop_ != nullptr) {
      g_main_loop_quit(gst_main_loop_);
    }
    if (gst_main_loop_thread_.joinable()) {
      gst_main_loop_thread_.join();
    }
    if (gst_main_loop_ != nullptr) {
      g_main_loop_unref(gst_main_loop_);
      gst_main_loop_ = nullptr;
    }
  }

 private:
  std::string video_config_path_;
  std::string base_host_;
  GMainLoop * gst_main_loop_{nullptr};
  std::thread gst_main_loop_thread_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  std::vector<std::shared_ptr<StreamPipeline>> pipelines_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscriptions_;
};

}  // namespace mr2_video_streaming

int main(int argc, char ** argv) {
  gst_init(&argc, &argv);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<mr2_video_streaming::VideoStreamingNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    std::fprintf(stderr, "video_streaming_node failed: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
