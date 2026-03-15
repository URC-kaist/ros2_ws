#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace mr2_video_streaming {

using json = nlohmann::json;

struct EncoderConfig {
  int bitrate_kbps{1500};
  int keyframe_interval{30};
  std::string speed_preset{"ultrafast"};
  std::string tune{"zerolatency"};
};

struct StreamConfig {
  std::string stream_id;
  std::string ros_topic;
  int udp_port{0};
  std::string ros_encoding;
  int width{0};
  int height{0};
  int framerate{15};
  EncoderConfig encoder;
};

class StreamPipeline {
 public:
  StreamPipeline(StreamConfig config, std::string base_host, rclcpp::Logger logger)
      : config_(std::move(config)),
        base_host_(std::move(base_host)),
        logger_(std::move(logger)),
        frame_duration_ns_(1000000000LL / std::max(1, config_.framerate)) {}

  ~StreamPipeline() { shutdown(); }

  void push_frame(const sensor_msgs::msg::Image & msg) {
    if (!validate_encoding(msg.encoding)) {
      return;
    }

    if (!ensure_pipeline(static_cast<int>(msg.width), static_cast<int>(msg.height))) {
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

    if (encoding != "rgb8" && encoding != "bgr8") {
      RCLCPP_WARN(
          logger_,
          "Stream %s received unsupported encoding %s",
          config_.stream_id.c_str(),
          encoding.c_str());
      return false;
    }

    return true;
  }

  bool ensure_pipeline(int frame_width, int frame_height) {
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
        << "! video/x-raw,format=I420 "
        << "! x264enc bitrate=" << config_.encoder.bitrate_kbps
        << " speed-preset=" << config_.encoder.speed_preset
        << " tune=" << config_.encoder.tune
        << " key-int-max=" << config_.encoder.keyframe_interval
        << " bframes=0 byte-stream=true threads=1 "
        << "! h264parse config-interval=1 "
        << "! rtph264pay pt=96 mtu=1200 config-interval=1 "
        << "! udpsink host=" << base_host_
        << " port=" << config_.udp_port
        << " sync=false async=false";

    GError * error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_description.str().c_str(), &error);
    if (pipeline_ == nullptr) {
      std::string message = error != nullptr ? error->message : "unknown error";
      if (error != nullptr) {
        g_error_free(error);
      }
      throw std::runtime_error("failed to create pipeline for " + config_.stream_id + ": " + message);
    }

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

    const GstStateChangeReturn state_change = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state_change == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("failed to start pipeline for stream " + config_.stream_id);
    }

    RCLCPP_INFO(
        logger_,
        "Started video stream %s topic=%s port=%d size=%dx%d fps=%d",
        config_.stream_id.c_str(),
        config_.ros_topic.c_str(),
        config_.udp_port,
        width_,
        height_,
        config_.framerate);
    return true;
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
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
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
  int width_{0};
  int height_{0};
  GstClockTime frame_duration_ns_{0};
  GstClockTime frame_index_{0};
};

StreamConfig parse_stream_config(const json & item) {
  StreamConfig config;
  config.stream_id = item.at("stream_id").get<std::string>();
  config.ros_topic = item.at("ros_topic").get<std::string>();
  config.udp_port = item.at("udp_port").get<int>();
  config.ros_encoding = item.at("ros_encoding").get<std::string>();
  config.width = item.value("width", 0);
  config.height = item.value("height", 0);
  config.framerate = std::max(1, item.value("framerate", 15));

  const json encoder = item.value("encoder", json::object());
  config.encoder.bitrate_kbps = std::max(100, encoder.value("bitrate_kbps", 1500));
  config.encoder.keyframe_interval = std::max(1, encoder.value("keyframe_interval", 30));
  config.encoder.speed_preset = encoder.value("speed_preset", std::string("ultrafast"));
  config.encoder.tune = encoder.value("tune", std::string("zerolatency"));

  if (config.ros_encoding != "rgb8" && config.ros_encoding != "bgr8") {
    throw std::runtime_error(
        "stream " + config.stream_id + " has unsupported ros_encoding " + config.ros_encoding);
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
  for (const auto & item : streams_json) {
    streams.push_back(parse_stream_config(item));
  }
  return streams;
}

class VideoStreamingNode : public rclcpp::Node {
 public:
  VideoStreamingNode()
      : rclcpp::Node("video_streaming"),
        video_config_path_(declare_parameter<std::string>("video_config_path", "")),
        base_host_(declare_parameter<std::string>("base_host", "127.0.0.1")) {
    if (video_config_path_.empty()) {
      throw std::runtime_error("video_config_path parameter is required");
    }

    const auto streams = load_streams_from_file(video_config_path_);
    for (const auto & stream : streams) {
      auto pipeline = std::make_shared<StreamPipeline>(stream, base_host_, get_logger());
      auto subscription = create_subscription<sensor_msgs::msg::Image>(
          stream.ros_topic,
          rclcpp::SensorDataQoS(),
          [pipeline](const sensor_msgs::msg::Image::SharedPtr msg) {
            pipeline->push_frame(*msg);
          });

      pipelines_.push_back(pipeline);
      subscriptions_.push_back(subscription);
      RCLCPP_INFO(
          get_logger(),
          "Configured video stream %s topic=%s port=%d encoding=%s",
          stream.stream_id.c_str(),
          stream.ros_topic.c_str(),
          stream.udp_port,
          stream.ros_encoding.c_str());
    }
  }

 private:
  std::string video_config_path_;
  std::string base_host_;
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
