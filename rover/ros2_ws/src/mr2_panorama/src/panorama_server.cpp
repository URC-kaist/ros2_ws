#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <camera_calibration_parsers/parse.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "mr2_action_interface/action/panorama_capture.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalize_degrees(double degrees)
{
  double normalized = std::fmod(degrees, 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }
  return normalized;
}

double shortest_angular_distance_degrees(double from, double to)
{
  double diff = normalize_degrees(to - from + 180.0) - 180.0;
  if (diff <= -180.0) {
    diff += 360.0;
  }
  return diff;
}

double radians_to_degrees(double radians)
{
  return radians * 180.0 / kPi;
}

std::string format_double(double value, int precision = 1)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(precision);
  out << value;
  return out.str();
}
}  // namespace

class PanoramaServer : public rclcpp::Node
{
public:
  using PanoramaCapture = mr2_action_interface::action::PanoramaCapture;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PanoramaCapture>;

  PanoramaServer()
  : Node("panorama_server"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    video_device_ = declare_parameter<std::string>("video_device", "/dev/videoFRONT");
    calibration_yaml_ = declare_parameter<std::string>(
      "calibration_yaml", default_calibration_yaml());
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    capture_width_ = declare_parameter<int>("capture_width", 848);
    capture_height_ = declare_parameter<int>("capture_height", 480);
    capture_fps_ = declare_parameter<double>("capture_fps", 10.0);
    default_angle_step_deg_ = declare_parameter<double>("default_angle_step_deg", 30.0);
    default_yaw_tolerance_deg_ = declare_parameter<double>("default_yaw_tolerance_deg", 5.0);
    default_timeout_sec_ = declare_parameter<double>("default_timeout_sec", 180.0);
    default_jpeg_quality_ = declare_parameter<int>("default_jpeg_quality", 90);

    action_server_ = rclcpp_action::create_server<PanoramaCapture>(
      this,
      "panorama_capture",
      std::bind(&PanoramaServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PanoramaServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&PanoramaServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "PanoramaCapture action server is up.");
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PanoramaCapture::Goal> goal)
  {
    bool expected = false;
    if (!goal_active_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Rejecting panorama goal because another goal is active.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    const double angle_step = goal->angle_step_deg > 0.0 ? goal->angle_step_deg : default_angle_step_deg_;
    const double tolerance = goal->yaw_tolerance_deg > 0.0 ?
      goal->yaw_tolerance_deg : default_yaw_tolerance_deg_;
    const double timeout = goal->timeout_sec > 0.0 ? goal->timeout_sec : default_timeout_sec_;

    if (!std::isfinite(angle_step) || angle_step <= 0.0 || angle_step > 180.0) {
      RCLCPP_WARN(get_logger(), "Rejecting panorama goal: invalid angle_step_deg %.3f", angle_step);
      goal_active_ = false;
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!std::isfinite(tolerance) || tolerance <= 0.0 || tolerance > 90.0) {
      RCLCPP_WARN(get_logger(), "Rejecting panorama goal: invalid yaw_tolerance_deg %.3f", tolerance);
      goal_active_ = false;
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!std::isfinite(timeout) || timeout <= 0.0) {
      RCLCPP_WARN(get_logger(), "Rejecting panorama goal: invalid timeout_sec %.3f", timeout);
      goal_active_ = false;
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  std::string default_calibration_yaml() const
  {
    try {
      return ament_index_cpp::get_package_share_directory("mr2_panorama") +
        "/config/front_camera_calibration.yaml";
    } catch (const std::exception & ex) {
      RCLCPP_WARN(
        get_logger(),
        "Could not resolve mr2_panorama calibration path from package index: %s",
        ex.what());
      return "front_camera_calibration.yaml";
    }
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel PanoramaCapture goal.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&PanoramaServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result = std::make_shared<PanoramaCapture::Result>();
    const auto clear_active = [this]() { goal_active_ = false; };

    const auto fail = [&](const std::string & message) {
      result->success = false;
      result->message = message;
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
      goal_handle->abort(result);
      clear_active();
    };

    const auto goal = goal_handle->get_goal();
    const double angle_step = goal->angle_step_deg > 0.0 ? goal->angle_step_deg : default_angle_step_deg_;
    const double yaw_tolerance = goal->yaw_tolerance_deg > 0.0 ?
      goal->yaw_tolerance_deg : default_yaw_tolerance_deg_;
    const double timeout_sec = goal->timeout_sec > 0.0 ? goal->timeout_sec : default_timeout_sec_;
    const int jpeg_quality = std::clamp<int>(
      goal->jpeg_quality > 0 ? goal->jpeg_quality : default_jpeg_quality_, 1, 100);
    const uint32_t expected_captures = static_cast<uint32_t>(std::ceil(360.0 / angle_step));
    result->expected_captures = expected_captures;

    sensor_msgs::msg::CameraInfo camera_info;
    cv::Mat camera_matrix;
    cv::Mat distortion_coeffs;
    double horizontal_fov_deg = 0.0;
    double vertical_fov_deg = 0.0;
    std::string calibration_error;
    if (!load_calibration(
        camera_info, camera_matrix, distortion_coeffs, horizontal_fov_deg, vertical_fov_deg,
        calibration_error))
    {
      fail(calibration_error);
      return;
    }

    if (!open_camera()) {
      fail("Failed to open V4L2 camera device: " + video_device_);
      return;
    }

    double start_yaw_deg = 0.0;
    std::string tf_error;
    if (!lookup_yaw_degrees(start_yaw_deg, tf_error)) {
      fail(tf_error);
      return;
    }
    start_yaw_deg = normalize_degrees(start_yaw_deg);

    RCLCPP_INFO(
      get_logger(),
      "Panorama capture started: step=%.1f deg tolerance=%.1f deg expected=%u start_yaw=%.1f deg",
      angle_step, yaw_tolerance, expected_captures, start_yaw_deg);

    std::vector<cv::Mat> frames;
    frames.reserve(expected_captures);
    auto start_time = now();
    rclcpp::Rate rate(10.0);

    while (rclcpp::ok() && frames.size() < expected_captures) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Panorama capture canceled";
        result->captures_taken = static_cast<uint32_t>(frames.size());
        result->expected_captures = expected_captures;
        goal_handle->canceled(result);
        clear_active();
        return;
      }

      const double elapsed = (now() - start_time).seconds();
      if (elapsed > timeout_sec) {
        result->captures_taken = static_cast<uint32_t>(frames.size());
        fail(
          "Panorama capture timed out after " + format_double(timeout_sec, 1) +
          " sec with " + std::to_string(frames.size()) + "/" +
          std::to_string(expected_captures) + " captures");
        return;
      }

      double current_yaw_deg = 0.0;
      if (!lookup_yaw_degrees(current_yaw_deg, tf_error)) {
        result->captures_taken = static_cast<uint32_t>(frames.size());
        fail(tf_error);
        return;
      }
      current_yaw_deg = normalize_degrees(current_yaw_deg);

      const double next_target_yaw_deg = normalize_degrees(
        start_yaw_deg + static_cast<double>(frames.size()) * angle_step);
      auto feedback = std::make_shared<PanoramaCapture::Feedback>();
      feedback->captures_taken = static_cast<uint32_t>(frames.size());
      feedback->expected_captures = expected_captures;
      feedback->current_yaw_deg = current_yaw_deg;
      feedback->next_target_yaw_deg = next_target_yaw_deg;
      goal_handle->publish_feedback(feedback);

      const double yaw_error = std::abs(
        shortest_angular_distance_degrees(current_yaw_deg, next_target_yaw_deg));
      if (yaw_error <= yaw_tolerance) {
        cv::Mat frame;
        std::string capture_error;
        if (!capture_undistorted_frame(camera_matrix, distortion_coeffs, frame, capture_error)) {
          result->captures_taken = static_cast<uint32_t>(frames.size());
          fail(capture_error);
          return;
        }
        frames.push_back(frame);
        RCLCPP_INFO(
          get_logger(), "Captured panorama frame %zu/%u at yaw %.1f deg",
          frames.size(), expected_captures, current_yaw_deg);
        std::this_thread::sleep_for(500ms);
      }

      rate.sleep();
    }

    cv::Mat panorama;
    std::string stitch_error;
    if (!stitch_frames(frames, panorama, stitch_error)) {
      result->captures_taken = static_cast<uint32_t>(frames.size());
      fail(stitch_error);
      return;
    }

    annotate_panorama(panorama, start_yaw_deg, horizontal_fov_deg, vertical_fov_deg);

    std::vector<uchar> jpeg;
    const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality};
    if (!cv::imencode(".jpg", panorama, jpeg, params) || jpeg.empty()) {
      result->captures_taken = static_cast<uint32_t>(frames.size());
      fail("Failed to JPEG-encode stitched panorama");
      return;
    }

    result->success = true;
    result->message = "Panorama capture succeeded";
    result->captures_taken = static_cast<uint32_t>(frames.size());
    result->expected_captures = expected_captures;
    result->panorama.header.stamp = now();
    result->panorama.header.frame_id = fixed_frame_;
    result->panorama.format = "jpeg";
    result->panorama.data.assign(jpeg.begin(), jpeg.end());
    goal_handle->succeed(result);
    clear_active();
  }

  bool load_calibration(
    sensor_msgs::msg::CameraInfo & camera_info,
    cv::Mat & camera_matrix,
    cv::Mat & distortion_coeffs,
    double & horizontal_fov_deg,
    double & vertical_fov_deg,
    std::string & error)
  {
    std::string camera_name;
    if (!camera_calibration_parsers::readCalibration(calibration_yaml_, camera_name, camera_info)) {
      error = "Failed to parse camera calibration YAML: " + calibration_yaml_;
      return false;
    }
    if (camera_info.k[0] <= 0.0 || camera_info.k[4] <= 0.0 ||
      camera_info.width == 0 || camera_info.height == 0)
    {
      error = "Camera calibration has invalid image size or focal length: " + calibration_yaml_;
      return false;
    }

    camera_matrix = cv::Mat(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        camera_matrix.at<double>(row, col) = camera_info.k[row * 3 + col];
      }
    }

    distortion_coeffs = cv::Mat(
      1, static_cast<int>(camera_info.d.size()), CV_64F, camera_info.d.data()).clone();

    horizontal_fov_deg = radians_to_degrees(
      2.0 * std::atan(static_cast<double>(camera_info.width) / (2.0 * camera_info.k[0])));
    vertical_fov_deg = radians_to_degrees(
      2.0 * std::atan(static_cast<double>(camera_info.height) / (2.0 * camera_info.k[4])));
    return true;
  }

  bool open_camera()
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (camera_.isOpened()) {
      return true;
    }

    if (!camera_.open(video_device_, cv::CAP_V4L2)) {
      return false;
    }
    camera_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(capture_width_));
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(capture_height_));
    camera_.set(cv::CAP_PROP_FPS, capture_fps_);

    cv::Mat warmup;
    for (int i = 0; i < 3; ++i) {
      camera_.read(warmup);
    }
    return camera_.isOpened();
  }

  bool capture_undistorted_frame(
    const cv::Mat & camera_matrix,
    const cv::Mat & distortion_coeffs,
    cv::Mat & undistorted,
    std::string & error)
  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    if (!camera_.isOpened()) {
      error = "V4L2 camera is not open: " + video_device_;
      return false;
    }

    cv::Mat frame;
    if (!camera_.read(frame) || frame.empty()) {
      error = "Failed to capture frame from V4L2 camera: " + video_device_;
      return false;
    }
    cv::undistort(frame, undistorted, camera_matrix, distortion_coeffs);
    if (undistorted.empty()) {
      error = "Failed to undistort captured frame";
      return false;
    }
    return true;
  }

  bool lookup_yaw_degrees(double & yaw_deg, std::string & error)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        fixed_frame_, base_frame_, tf2::TimePointZero, 500ms);
      yaw_deg = radians_to_degrees(tf2::getYaw(transform.transform.rotation));
      return true;
    } catch (const tf2::TransformException & ex) {
      error = "Failed to lookup TF yaw from " + fixed_frame_ + " to " + base_frame_ + ": " + ex.what();
      return false;
    }
  }

  bool stitch_frames(const std::vector<cv::Mat> & frames, cv::Mat & panorama, std::string & error)
  {
    if (frames.size() < 2) {
      error = "Need at least two captured frames to stitch a panorama";
      return false;
    }

    cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(cv::Stitcher::PANORAMA);
    const cv::Stitcher::Status status = stitcher->stitch(frames, panorama);
    if (status != cv::Stitcher::OK || panorama.empty()) {
      error = "OpenCV Stitcher failed with status " + std::to_string(static_cast<int>(status));
      return false;
    }
    return true;
  }

  void annotate_panorama(
    cv::Mat & panorama,
    double start_yaw_deg,
    double horizontal_fov_deg,
    double vertical_fov_deg)
  {
    const int width = panorama.cols;
    const int height = panorama.rows;
    const int top = std::max(34, height / 24);
    const int bottom = height - std::max(28, height / 28);
    const cv::Scalar white(255, 255, 255);
    const cv::Scalar black(0, 0, 0);
    const cv::Scalar accent(0, 220, 255);

    auto text = [&](const std::string & label, cv::Point origin, double scale, const cv::Scalar & color) {
      cv::putText(panorama, label, origin + cv::Point(1, 1), cv::FONT_HERSHEY_SIMPLEX, scale, black, 3, cv::LINE_AA);
      cv::putText(panorama, label, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
    };

    cv::line(panorama, {0, top}, {width - 1, top}, white, 2, cv::LINE_AA);
    for (int deg = 0; deg <= 360; deg += 15) {
      const int x = std::clamp(static_cast<int>(std::round(width * deg / 360.0)), 0, width - 1);
      const bool major = deg % 30 == 0;
      const int tick = major ? 18 : 10;
      cv::line(panorama, {x, top - tick / 2}, {x, top + tick / 2}, white, major ? 2 : 1, cv::LINE_AA);
      if (major && deg < 360) {
        const double yaw = normalize_degrees(start_yaw_deg + deg);
        text(format_double(yaw, 0), {std::max(0, x - 16), top + 34}, 0.45, white);
      }
    }
    text("360 deg horizontal panorama", {12, top - 12}, 0.55, accent);
    text("camera HFOV " + format_double(horizontal_fov_deg, 1) + " deg", {12, top + 58}, 0.45, accent);

    struct Cardinal
    {
      const char * label;
      double yaw_deg;
    };
    const Cardinal cardinals[] = {
      {"E", 0.0},
      {"N", 90.0},
      {"W", 180.0},
      {"S", 270.0},
    };
    for (const auto & cardinal : cardinals) {
      const double offset = normalize_degrees(cardinal.yaw_deg - start_yaw_deg);
      const int x = std::clamp(static_cast<int>(std::round(width * offset / 360.0)), 0, width - 1);
      cv::line(panorama, {x, 0}, {x, height - 1}, accent, 2, cv::LINE_AA);
      text(cardinal.label, {std::max(0, x - 12), std::max(24, top - 36)}, 0.9, accent);
    }

    const int bracket_x = width - std::max(28, width / 36);
    const int bracket_top = std::max(top + 70, height / 4);
    const int bracket_bottom = std::min(bottom - 10, 3 * height / 4);
    cv::line(panorama, {bracket_x, bracket_top}, {bracket_x, bracket_bottom}, accent, 2, cv::LINE_AA);
    cv::line(panorama, {bracket_x - 18, bracket_top}, {bracket_x + 18, bracket_top}, accent, 2, cv::LINE_AA);
    cv::line(panorama, {bracket_x - 18, bracket_bottom}, {bracket_x + 18, bracket_bottom}, accent, 2, cv::LINE_AA);
    text("VFOV " + format_double(vertical_fov_deg, 1) + " deg", {std::max(4, bracket_x - 170), bracket_top - 12}, 0.5, accent);
  }

  rclcpp_action::Server<PanoramaCapture>::SharedPtr action_server_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::atomic_bool goal_active_{false};

  std::mutex camera_mutex_;
  cv::VideoCapture camera_;

  std::string video_device_;
  std::string calibration_yaml_;
  std::string fixed_frame_;
  std::string base_frame_;
  int capture_width_;
  int capture_height_;
  double capture_fps_;
  double default_angle_step_deg_;
  double default_yaw_tolerance_deg_;
  double default_timeout_sec_;
  int default_jpeg_quality_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PanoramaServer>());
  rclcpp::shutdown();
  return 0;
}
