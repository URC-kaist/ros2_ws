#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cctype>
#include <cstdlib>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
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

#include "mr2_latency_msgs/srv/acquire_video_stream_lease.hpp"
#include "mr2_latency_msgs/srv/release_video_stream_lease.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace mr2_video_streaming {

using json = nlohmann::json;

constexpr int kUdpSendBufferBytes = 65536;
constexpr int kMinV4L2RestartDelaySeconds = 1;
constexpr int kMaxV4L2RestartDelaySeconds = 30;
constexpr int kV4L2StableRuntimeSeconds = 5;

enum class StreamSourceType { RosTopic, V4L2 };

struct EncoderConfig {
  std::string type{"x264"};
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
  int v4l2_capture_width{0};
  int v4l2_capture_height{0};
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

bool is_jetson_hardware_encoder(const std::string & encoder_type) {
  return encoder_type == "nvv4l2h264enc" || encoder_type == "jetson_h264" ||
         encoder_type == "jetson";
}

bool is_supported_encoder_type(const std::string & encoder_type) {
  return encoder_type == "x264" || encoder_type == "x264enc" ||
         is_jetson_hardware_encoder(encoder_type);
}

bool is_valid_lease_owner(const std::string & value) {
  if (value.empty() || value.size() > 128) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char ch) {
    return std::isalnum(ch) || ch == '-' || ch == '_' || ch == '.';
  });
}

std::string trim_copy(const std::string & value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::set<std::string> parse_disabled_stream_ids(const std::string & value) {
  std::set<std::string> stream_ids;
  std::stringstream input(value);
  std::string item;
  while (std::getline(input, item, ',')) {
    item = trim_copy(item);
    if (!item.empty()) {
      stream_ids.insert(item);
    }
  }
  return stream_ids;
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

  const std::string & stream_id() const { return config_.stream_id; }

  bool is_enabled() const { return enabled_; }

  bool set_enabled(bool enabled) {
    if (enabled_ == enabled) {
      return false;
    }

    enabled_ = enabled;
    if (!enabled_) {
      shutdown();
      RCLCPP_INFO(logger_, "Disabled video stream %s", config_.stream_id.c_str());
      return true;
    }

    stopping_ = false;
    v4l2_restart_failures_ = 0;
    next_v4l2_restart_at_ = std::chrono::steady_clock::time_point::min();
    start_if_needed();
    RCLCPP_INFO(logger_, "Enabled video stream %s", config_.stream_id.c_str());
    return true;
  }

  void start_if_needed() {
    if (!enabled_ || config_.source_type != StreamSourceType::V4L2 || pipeline_ != nullptr ||
        gst_child_pid_ > 0) {
      return;
    }
    if (next_v4l2_restart_at_ != std::chrono::steady_clock::time_point::max() &&
        std::chrono::steady_clock::now() < next_v4l2_restart_at_) {
      return;
    }
    start_v4l2_pipeline();
  }

  void poll_runtime() {
    if (!enabled_ || config_.source_type != StreamSourceType::V4L2 || stopping_) {
      return;
    }

    if (gst_child_pid_ > 0) {
      int status = 0;
      const pid_t result = waitpid(gst_child_pid_, &status, WNOHANG);
      if (result == 0) {
        const auto now = std::chrono::steady_clock::now();
        if (!v4l2_child_reported_stable_ &&
            now - gst_child_started_at_ >= std::chrono::seconds(kV4L2StableRuntimeSeconds)) {
          v4l2_restart_failures_ = 0;
          v4l2_child_reported_stable_ = true;
        }
        return;
      }

      if (result == gst_child_pid_) {
        std::string reason;
        if (WIFEXITED(status)) {
          reason = "exited with code " + std::to_string(WEXITSTATUS(status));
        } else if (WIFSIGNALED(status)) {
          reason = "exited on signal " + std::to_string(WTERMSIG(status));
        } else {
          reason = "exited unexpectedly";
        }
        gst_child_pid_ = -1;
        schedule_v4l2_restart("V4L2 sender child for stream " + config_.stream_id + " " + reason);
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
        schedule_v4l2_restart("V4L2 sender child for stream " + config_.stream_id + " disappeared");
      }
    }

    if (gst_child_pid_ <= 0 && std::chrono::steady_clock::now() >= next_v4l2_restart_at_) {
      try {
        start_v4l2_pipeline();
      } catch (const std::exception & error) {
        schedule_v4l2_restart(
            "failed to restart V4L2 sender for stream " + config_.stream_id + ": " + error.what());
      }
    }
  }

  void push_frame(const sensor_msgs::msg::Image & msg) {
    if (!enabled_ || config_.source_type != StreamSourceType::RosTopic) {
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
  static void append_live_leaky_queue(std::vector<std::string> & args) {
    args.insert(
        args.end(),
        {
            "!",
            "queue",
            "leaky=downstream",
            "max-size-buffers=1",
            "max-size-bytes=0",
            "max-size-time=0",
        });
  }

  bool validate_encoding(const std::string & encoding) {
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
    if (is_jetson_hardware_encoder(config_.encoder.type)) {
      branch << "! nvvidconv "
             << "! video/x-raw(memory:NVMM),format=NV12 "
             << "! nvv4l2h264enc bitrate=" << (config_.encoder.bitrate_kbps * 1000)
             << " iframeinterval=" << config_.encoder.keyframe_interval
             << " insert-sps-pps=true maxperf-enable=true control-rate=1 ";
    } else {
      branch << "! x264enc bitrate=" << config_.encoder.bitrate_kbps
             << " speed-preset=" << config_.encoder.speed_preset
             << " tune=" << config_.encoder.tune
             << " key-int-max=" << config_.encoder.keyframe_interval
             << " bframes=0 byte-stream=true threads=1 ";
    }
    branch << "! h264parse config-interval=1 "
           << "! rtph264pay pt=96 mtu=1200 config-interval=1 "
           << "! udpsink host=" << quote_gstreamer_string(base_host_)
           << " port=" << config_.udp_port
           << " buffer-size=" << kUdpSendBufferBytes
           << " sync=false async=false";
    return branch.str();
  }

  std::string build_raw_output_caps() const {
    std::ostringstream caps;
    caps << "video/x-raw,format=I420";
    if (output_width_ > 0) {
      caps << ",width=" << output_width_;
    }
    if (output_height_ > 0) {
      caps << ",height=" << output_height_;
    }
    if (config_.framerate > 0) {
      caps << ",framerate=" << std::max(1, config_.framerate) << "/1";
    }
    return caps.str();
  }

  std::string build_nvmm_output_caps() const {
    std::ostringstream caps;
    caps << "video/x-raw(memory:NVMM),format=NV12";
    if (output_width_ > 0) {
      caps << ",width=" << output_width_;
    }
    if (output_height_ > 0) {
      caps << ",height=" << output_height_;
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
    const int capture_width =
        config_.v4l2_capture_width > 0 ? config_.v4l2_capture_width : config_.width;
    const int capture_height =
        config_.v4l2_capture_height > 0 ? config_.v4l2_capture_height : config_.height;

    caps << (is_jpeg ? "image/jpeg" : "video/x-raw");
    if (!is_jpeg && !pixel_format.empty()) {
      caps << ",format=" << pixel_format;
    }
    if (capture_width > 0) {
      caps << ",width=" << capture_width;
    }
    if (capture_height > 0) {
      caps << ",height=" << capture_height;
    }
    if (config_.framerate > 0) {
      caps << ",framerate=" << std::max(1, config_.framerate) << "/1";
    }
    return caps.str();
  }

  bool ensure_ros_pipeline(int frame_width, int frame_height) {
    if (pipeline_ != nullptr) {
      if (frame_width != input_width_ || frame_height != input_height_) {
        RCLCPP_WARN(
            logger_,
            "Stream %s received unexpected frame dimensions %dx%d (expected %dx%d)",
            config_.stream_id.c_str(),
            frame_width,
            frame_height,
            input_width_,
            input_height_);
        return false;
      }
      return true;
    }

    input_width_ = frame_width;
    input_height_ = frame_height;
    output_width_ = config_.width > 0 ? config_.width : frame_width;
    output_height_ = config_.height > 0 ? config_.height : frame_height;

    if (output_width_ != frame_width || output_height_ != frame_height) {
      RCLCPP_INFO(
          logger_,
          "Stream %s scaling ROS frames from %dx%d to configured size %dx%d",
          config_.stream_id.c_str(),
          frame_width,
          frame_height,
          output_width_,
          output_height_);
    }

    std::ostringstream pipeline_description;
    pipeline_description
        << "appsrc name=src is-live=true format=time block=false do-timestamp=false max-buffers=1 "
        << "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
        << "! videoconvert "
        << "! videoscale "
        << "! " << build_raw_output_caps() << " "
        << build_encoded_sink_branch();

    start_pipeline(pipeline_description.str(), true);

    RCLCPP_INFO(
        logger_,
        "Started ROS video stream %s topic=%s port=%d size=%dx%d fps=%d",
        config_.stream_id.c_str(),
        config_.ros_topic.c_str(),
        config_.udp_port,
        output_width_,
        output_height_,
        config_.framerate);
    return true;
  }

  void start_v4l2_pipeline() {
    if (gst_child_pid_ > 0) {
      return;
    }
    if (!std::filesystem::exists(config_.v4l2_device)) {
      schedule_v4l2_restart(
          "V4L2 device " + config_.v4l2_device + " for stream " + config_.stream_id + " is not present");
      return;
    }

    const int capture_width =
        config_.v4l2_capture_width > 0 ? config_.v4l2_capture_width : config_.width;
    const int capture_height =
        config_.v4l2_capture_height > 0 ? config_.v4l2_capture_height : config_.height;
    output_width_ = config_.width > 0 ? config_.width : capture_width;
    output_height_ = config_.height > 0 ? config_.height : capture_height;

    const std::string pixel_format = uppercase(config_.v4l2_pixel_format);
    const bool is_jpeg = pixel_format == "MJPG" || pixel_format == "JPEG";

    std::vector<std::string> args{
        "gst-launch-1.0",
        "-q",
        "v4l2src",
        "device=" + config_.v4l2_device,
        "do-timestamp=true",
    };
    const std::string source_caps = build_v4l2_source_caps();
    if (!source_caps.empty()) {
      args.push_back("!");
      args.push_back(source_caps);
    }
    append_live_leaky_queue(args);
    if (is_jetson_hardware_encoder(config_.encoder.type)) {
      if (is_jpeg) {
        args.insert(args.end(), {"!", "nvv4l2decoder", "mjpeg=true"});
      }
      args.insert(
          args.end(),
          {
              "!",
              "nvvidconv",
              "!",
              build_nvmm_output_caps(),
              "!",
              "nvv4l2h264enc",
              "bitrate=" + std::to_string(config_.encoder.bitrate_kbps * 1000),
              "iframeinterval=" + std::to_string(config_.encoder.keyframe_interval),
              "insert-sps-pps=true",
              "maxperf-enable=true",
              "control-rate=1",
          });
    } else {
      if (is_jpeg) {
        args.push_back("!");
        args.push_back("jpegdec");
      }
      args.insert(
          args.end(),
          {"!", "videoconvert", "!", "videoscale", "!", build_raw_output_caps()});
      args.insert(
          args.end(),
          {
              "!",
              "x264enc",
              "bitrate=" + std::to_string(config_.encoder.bitrate_kbps),
              "speed-preset=" + config_.encoder.speed_preset,
              "tune=" + config_.encoder.tune,
              "key-int-max=" + std::to_string(config_.encoder.keyframe_interval),
              "bframes=0",
              "byte-stream=true",
              "threads=1",
          });
    }
    args.insert(
        args.end(),
        {
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
            "buffer-size=" + std::to_string(kUdpSendBufferBytes),
            "sync=false",
            "async=false",
        });

    start_gst_launch_child(args);
    gst_child_started_at_ = std::chrono::steady_clock::now();
    v4l2_child_reported_stable_ = false;
    next_v4l2_restart_at_ = std::chrono::steady_clock::time_point::max();

    std::ostringstream details;
    details << "Started V4L2 video stream " << config_.stream_id
            << " device=" << config_.v4l2_device
            << " port=" << config_.udp_port;
    if (capture_width > 0 && capture_height > 0) {
      details << " capture=" << capture_width << "x" << capture_height;
    }
    if (output_width_ > 0 && output_height_ > 0) {
      details << " output=" << output_width_ << "x" << output_height_;
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
          input_width_,
          "height",
          G_TYPE_INT,
          input_height_,
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

  void schedule_v4l2_restart(const std::string & reason) {
    v4l2_restart_failures_ = std::min(v4l2_restart_failures_ + 1, 6);
    const int exponential_delay = kMinV4L2RestartDelaySeconds << (v4l2_restart_failures_ - 1);
    const int delay_seconds = std::min(exponential_delay, kMaxV4L2RestartDelaySeconds);
    next_v4l2_restart_at_ =
        std::chrono::steady_clock::now() + std::chrono::seconds(delay_seconds);
    RCLCPP_WARN(
        logger_,
        "%s; retrying in %d s",
        reason.c_str(),
        delay_seconds);
  }

  StreamConfig config_;
  std::string base_host_;
  rclcpp::Logger logger_;
  GstElement * pipeline_{nullptr};
  GstElement * appsrc_{nullptr};
  GstBus * bus_{nullptr};
  guint bus_watch_id_{0};
  pid_t gst_child_pid_{-1};
  int input_width_{0};
  int input_height_{0};
  int output_width_{0};
  int output_height_{0};
  GstClockTime frame_duration_ns_{0};
  GstClockTime frame_index_{0};
  bool stopping_{false};
  bool enabled_{true};
  int v4l2_restart_failures_{0};
  bool v4l2_child_reported_stable_{false};
  std::chrono::steady_clock::time_point gst_child_started_at_{};
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
  config.encoder.type = encoder.value("type", std::string("x264"));
  if (!is_supported_encoder_type(config.encoder.type)) {
    throw std::runtime_error(
        "stream " + config.stream_id + " has unsupported encoder.type " + config.encoder.type);
  }
  config.encoder.bitrate_kbps = std::max(100, encoder.value("bitrate_kbps", 1500));
  config.encoder.keyframe_interval = std::max(1, encoder.value("keyframe_interval", 30));
  config.encoder.speed_preset = encoder.value("speed_preset", std::string("ultrafast"));
  config.encoder.tune = encoder.value("tune", std::string("zerolatency"));

  if (!item.contains("source")) {
    throw std::runtime_error("stream " + config.stream_id + " is missing source");
  }

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
    config.v4l2_capture_width =
        source.value("capture_width", item.value("capture_width", config.width));
    config.v4l2_capture_height =
        source.value("capture_height", item.value("capture_height", config.height));
  } else {
    throw std::runtime_error(
        "stream " + config.stream_id + " has unsupported source.type " + source_type);
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
        base_host_(declare_parameter<std::string>("base_host", default_base_host())),
        stream_lease_timeout_s_(declare_parameter<double>("stream_lease_timeout_s", 60.0)),
        disabled_stream_ids_(parse_disabled_stream_ids(
            declare_parameter<std::string>("disabled_stream_ids", ""))) {
    if (video_config_path_.empty()) {
      throw std::runtime_error("video_config_path parameter is required");
    }

    std::unique_ptr<GMainLoop, decltype(&g_main_loop_unref)> gst_main_loop(
        g_main_loop_new(nullptr, FALSE), &g_main_loop_unref);
    if (!gst_main_loop) {
      throw std::runtime_error("failed to allocate GMainLoop");
    }

    const auto streams = load_streams_from_file(video_config_path_);
    for (const auto & stream : streams) {
      auto pipeline = std::make_shared<StreamPipeline>(stream, base_host_, get_logger());
      if (disabled_stream_ids_.count(stream.stream_id) > 0) {
        pipeline->set_enabled(false);
      }

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
      const std::string service_name =
          std::string(get_fully_qualified_name()) + "/streams/" + stream.stream_id + "/set_enabled";
      auto service = create_service<std_srvs::srv::SetBool>(
          service_name,
          [this, pipeline, stream_id = stream.stream_id](
              const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
              std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            expire_stream_lease_if_needed();
            if (stream_lease_active()) {
              response->success = false;
              response->message = "stream configuration is locked by latency trial " +
                stream_lease_owner_;
              return;
            }
            const bool changed = pipeline->set_enabled(request->data);
            response->success = true;
            response->message =
                "stream " + stream_id + (pipeline->is_enabled() ? " enabled" : " disabled");
            if (!changed) {
              response->message += " (already in requested state)";
            }
          });
      stream_enable_services_.push_back(service);

      std::ostringstream details;
      details << "Configured video stream " << stream.stream_id
              << " source=" << source_type_to_string(stream.source_type)
              << " port=" << stream.udp_port
              << " service=" << service_name;
      if (stream.source_type == StreamSourceType::RosTopic) {
        details << " topic=" << stream.ros_topic
                << " encoding=" << stream.ros_encoding;
      } else {
        details << " device=" << stream.v4l2_device;
        if (!stream.v4l2_pixel_format.empty()) {
          details << " pixel_format=" << stream.v4l2_pixel_format;
        }
        if (stream.v4l2_capture_width > 0 && stream.v4l2_capture_height > 0) {
          details << " capture=" << stream.v4l2_capture_width << "x"
                  << stream.v4l2_capture_height;
        }
      }
      if (stream.width > 0 && stream.height > 0) {
        details << " output=" << stream.width << "x" << stream.height;
      }
      RCLCPP_INFO(get_logger(), "%s", details.str().c_str());
    }

    acquire_stream_lease_service_ =
      create_service<mr2_latency_msgs::srv::AcquireVideoStreamLease>(
        std::string(get_fully_qualified_name()) + "/acquire_stream_lease",
        [this](
          const std::shared_ptr<mr2_latency_msgs::srv::AcquireVideoStreamLease::Request> request,
          std::shared_ptr<mr2_latency_msgs::srv::AcquireVideoStreamLease::Response> response) {
          acquire_stream_lease(*request, *response);
        });
    release_stream_lease_service_ =
      create_service<mr2_latency_msgs::srv::ReleaseVideoStreamLease>(
        std::string(get_fully_qualified_name()) + "/release_stream_lease",
        [this](
          const std::shared_ptr<mr2_latency_msgs::srv::ReleaseVideoStreamLease::Request> request,
          std::shared_ptr<mr2_latency_msgs::srv::ReleaseVideoStreamLease::Response> response) {
          release_stream_lease(*request, *response);
        });

    monitor_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
      expire_stream_lease_if_needed();
      for (const auto & pipeline : pipelines_) {
        pipeline->poll_runtime();
      }
    });

    gst_main_loop_ = gst_main_loop.get();
    gst_main_loop_thread_ = std::thread([this]() {
      g_main_loop_run(gst_main_loop_);
    });
    gst_main_loop.release();
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
  bool stream_lease_active() const {
    return !stream_lease_owner_.empty() &&
           std::chrono::steady_clock::now() < stream_lease_expires_at_;
  }

  void expire_stream_lease_if_needed() {
    if (!stream_lease_owner_.empty() &&
        std::chrono::steady_clock::now() >= stream_lease_expires_at_) {
      const auto owner = stream_lease_owner_;
      restore_stream_lease();
      RCLCPP_WARN(get_logger(), "Expired video stream lease for %s", owner.c_str());
    }
  }

  std::shared_ptr<StreamPipeline> find_pipeline(const std::string & stream_id) const {
    const auto found = std::find_if(
      pipelines_.begin(), pipelines_.end(), [&stream_id](const auto & pipeline) {
        return pipeline->stream_id() == stream_id;
      });
    return found == pipelines_.end() ? nullptr : *found;
  }

  void acquire_stream_lease(
    const mr2_latency_msgs::srv::AcquireVideoStreamLease::Request & request,
    mr2_latency_msgs::srv::AcquireVideoStreamLease::Response & response) {
    expire_stream_lease_if_needed();
    if (!is_valid_lease_owner(request.owner_id)) {
      response.message = "invalid lease owner";
      return;
    }
    if (stream_lease_active()) {
      response.message = "video streams are already leased by " + stream_lease_owner_;
      return;
    }
    if (request.stream_ids.empty()) {
      response.message = "at least one stream is required";
      return;
    }

    std::set<std::string> selected;
    for (const auto & stream_id : request.stream_ids) {
      if (!find_pipeline(stream_id)) {
        response.message = "unknown stream " + stream_id;
        return;
      }
      selected.insert(stream_id);
    }

    stream_lease_previous_states_.clear();
    for (const auto & pipeline : pipelines_) {
      stream_lease_previous_states_[pipeline->stream_id()] = pipeline->is_enabled();
      if (pipeline->is_enabled()) {
        response.previous_stream_ids.push_back(pipeline->stream_id());
      }
      pipeline->set_enabled(selected.count(pipeline->stream_id()) > 0);
      if (pipeline->is_enabled()) {
        response.effective_stream_ids.push_back(pipeline->stream_id());
      }
    }
    stream_lease_owner_ = request.owner_id;
    const double requested_timeout = request.lease_timeout_s > 0.0 ?
      request.lease_timeout_s : stream_lease_timeout_s_;
    const double timeout_s = std::clamp(requested_timeout, 5.0, 300.0);
    stream_lease_expires_at_ = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int64_t>(timeout_s * 1000.0));
    response.accepted = true;
    response.message = "video stream lease acquired";
  }

  std::vector<std::string> restore_stream_lease() {
    std::vector<std::string> restored;
    for (const auto & pipeline : pipelines_) {
      const auto previous = stream_lease_previous_states_.find(pipeline->stream_id());
      if (previous == stream_lease_previous_states_.end()) {
        continue;
      }
      pipeline->set_enabled(previous->second);
      if (pipeline->is_enabled()) {
        restored.push_back(pipeline->stream_id());
      }
    }
    stream_lease_owner_.clear();
    stream_lease_previous_states_.clear();
    stream_lease_expires_at_ = std::chrono::steady_clock::time_point::min();
    return restored;
  }

  void release_stream_lease(
    const mr2_latency_msgs::srv::ReleaseVideoStreamLease::Request & request,
    mr2_latency_msgs::srv::ReleaseVideoStreamLease::Response & response) {
    expire_stream_lease_if_needed();
    if (stream_lease_owner_.empty()) {
      response.message = "no active video stream lease";
      return;
    }
    if (request.owner_id != stream_lease_owner_) {
      response.message = "lease owner does not match";
      return;
    }
    response.restored_stream_ids = restore_stream_lease();
    response.released = true;
    response.message = "video stream lease released";
  }

  static std::string default_base_host() {
    const char * env_base_ip = std::getenv("MR2_BASE_IP");
    if (env_base_ip != nullptr && env_base_ip[0] != '\0') {
      return env_base_ip;
    }
    const auto env_file_base_ip = read_env_file_value("MR2_BASE_IP");
    if (!env_file_base_ip.empty()) {
      return env_file_base_ip;
    }
    throw std::runtime_error("MR2_BASE_IP environment variable is required");
  }

  static std::string read_env_file_value(const std::string & name) {
    std::error_code ec;
    auto directory = std::filesystem::current_path(ec);
    if (ec) {
      return "";
    }

    while (true) {
      const auto env_path = directory / ".env";
      if (std::filesystem::is_regular_file(env_path, ec)) {
        std::ifstream input(env_path);
        std::string line;
        while (std::getline(input, line)) {
          const auto first = line.find_first_not_of(" \t");
          if (first == std::string::npos || line[first] == '#') {
            continue;
          }
          const auto equals = line.find('=', first);
          if (equals == std::string::npos) {
            continue;
          }
          const auto key_end = line.find_last_not_of(" \t", equals - 1);
          const auto key = line.substr(first, key_end - first + 1);
          if (key != name) {
            continue;
          }
          const auto value_start = line.find_first_not_of(" \t", equals + 1);
          if (value_start == std::string::npos) {
            return "";
          }
          const auto value_end = line.find_last_not_of(" \t\r");
          auto value = line.substr(value_start, value_end - value_start + 1);
          if (value.size() >= 2 &&
              ((value.front() == '"' && value.back() == '"') ||
               (value.front() == '\'' && value.back() == '\''))) {
            value = value.substr(1, value.size() - 2);
          }
          return value;
        }
      }

      const auto parent = directory.parent_path();
      if (parent == directory) {
        break;
      }
      directory = parent;
    }
    return "";
  }

  std::string video_config_path_;
  std::string base_host_;
  double stream_lease_timeout_s_{60.0};
  std::set<std::string> disabled_stream_ids_;
  std::string stream_lease_owner_;
  std::map<std::string, bool> stream_lease_previous_states_;
  std::chrono::steady_clock::time_point stream_lease_expires_at_{
    std::chrono::steady_clock::time_point::min()};
  GMainLoop * gst_main_loop_{nullptr};
  std::thread gst_main_loop_thread_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  std::vector<std::shared_ptr<StreamPipeline>> pipelines_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscriptions_;
  std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> stream_enable_services_;
  rclcpp::Service<mr2_latency_msgs::srv::AcquireVideoStreamLease>::SharedPtr
    acquire_stream_lease_service_;
  rclcpp::Service<mr2_latency_msgs::srv::ReleaseVideoStreamLease>::SharedPtr
    release_stream_lease_service_;
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
