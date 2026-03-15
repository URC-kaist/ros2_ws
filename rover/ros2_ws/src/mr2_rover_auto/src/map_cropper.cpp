#include <ament_index_cpp/get_package_share_directory.hpp>

#include <mr2_rover_auto/srv/crop_map.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <robot_localization/srv/from_ll.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <optional>
#include <regex>
#include <sstream>
#include <string>

namespace
{

constexpr double kEpsilon = 1e-9;

std::optional<std::string> read_file(const std::string & path)
{
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    return std::nullopt;
  }

  std::ostringstream oss;
  oss << ifs.rdbuf();
  return oss.str();
}

std::optional<double> extract_scalar(const std::string & content, const std::string & key)
{
  static const std::string number_pattern =
    R"(([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?))";
  const std::regex re("\"" + key + "\"\\s*:\\s*" + number_pattern);
  std::smatch match;
  if (!std::regex_search(content, match, re) || match.size() < 2) {
    return std::nullopt;
  }

  try {
    return std::stod(match[1].str());
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<std::array<double, 4>> extract_bbox(const std::string & content)
{
  static const std::string number_pattern =
    R"(([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?))";
  const std::regex re(
    "\\\"bbox\\\"\\s*:\\s*\\[\\s*" + number_pattern +
    "\\s*,\\s*" + number_pattern +
    "\\s*,\\s*" + number_pattern +
    "\\s*,\\s*" + number_pattern +
    "\\s*\\]");

  std::smatch match;
  if (!std::regex_search(content, match, re) || match.size() < 5) {
    return std::nullopt;
  }

  try {
    return std::array<double, 4>{
      std::stod(match[1].str()), std::stod(match[2].str()),
      std::stod(match[3].str()), std::stod(match[4].str())};
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

bool ends_with(const std::string & value, const std::string & suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool is_digits_only(const std::string & text)
{
  return !text.empty() &&
         std::all_of(text.begin(), text.end(), [](unsigned char c) {return std::isdigit(c) != 0;});
}

}  // namespace

class MapCropper : public rclcpp::Node
{
public:
  using CropMap = mr2_rover_auto::srv::CropMap;
  using LoadMap = nav2_msgs::srv::LoadMap;
  using FromLL = robot_localization::srv::FromLL;

  MapCropper()
  : Node("map_cropper"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_, this, true)
  {
    const auto pkg_share = ament_index_cpp::get_package_share_directory("mr2_rover_auto");
    std::string package_root = pkg_share;

    const auto share_path = std::filesystem::path(pkg_share);
    const auto workspace_root =
      share_path.parent_path().parent_path().parent_path().parent_path();
    const auto source_package_path = workspace_root / "src" / "mr2_rover_auto";
    const auto source_map_candidate = source_package_path / "maps" / "lidar_enu_4km_slope.png";
    if (std::filesystem::exists(source_map_candidate)) {
      package_root = source_package_path.string();
    }

    source_map_path_ = this->declare_parameter<std::string>(
      "source_map_path",
      package_root + "/maps/lidar_enu_4km_slope.png");
    metadata_path_ = this->declare_parameter<std::string>(
      "metadata_path",
      package_root + "/maps/lidar_enu_4km_metadata.json");
    output_dir_ = this->declare_parameter<std::string>(
      "output_dir",
      package_root + "/maps/crops");
    output_prefix_ = this->declare_parameter<std::string>("output_prefix", "crop_");
    service_name_ = this->declare_parameter<std::string>("service_name", "map_cropper/crop_map");
    map_server_service_name_ = this->declare_parameter<std::string>(
      "map_server_service_name", "/map_server/load_map");
    from_ll_service_name_ = this->declare_parameter<std::string>(
      "from_ll_service", "fromLL");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    tf_timeout_s_ = this->declare_parameter<double>("tf_timeout_s", 0.2);
    from_ll_wait_timeout_s_ = this->declare_parameter<double>("from_ll_wait_timeout_s", 2.0);
    from_ll_response_timeout_s_ = this->declare_parameter<double>("from_ll_response_timeout_s", 5.0);

    roi_padding_m_ = this->declare_parameter<double>("roi_padding_m", 100.0);
    load_map_wait_timeout_s_ = this->declare_parameter<double>("load_map_wait_timeout_s", 3.0);
    load_map_response_timeout_s_ = this->declare_parameter<double>("load_map_response_timeout_s", 10.0);

    yaml_mode_ = this->declare_parameter<std::string>("yaml_mode", "scale");
    yaml_negate_ = this->declare_parameter<int>("yaml_negate", 1);
    yaml_occupied_thresh_ = this->declare_parameter<double>("yaml_occupied_thresh", 1.0);
    yaml_free_thresh_ = this->declare_parameter<double>("yaml_free_thresh", 0.0);

    if (!load_source_map()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize map_cropper due to source map loading errors");
      initialized_ = false;
    } else {
      initialized_ = true;
      RCLCPP_INFO(
        get_logger(),
        "map_cropper ready: image=%s metadata=%s output_dir=%s service=%s",
        source_map_path_.c_str(), metadata_path_.c_str(), output_dir_.c_str(), service_name_.c_str());
    }

    service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    crop_service_ = this->create_service<CropMap>(
      service_name_,
      std::bind(
        &MapCropper::on_crop_request, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
      rmw_qos_profile_services_default,
      service_group_);

    map_server_client_ = this->create_client<LoadMap>(
      map_server_service_name_, rmw_qos_profile_services_default, client_group_);
    from_ll_client_ = this->create_client<FromLL>(
      from_ll_service_name_, rmw_qos_profile_services_default, client_group_);
  }

private:
  struct MapMetadata
  {
    double min_x{};
    double min_y{};
    double max_x{};
    double max_y{};
    double resolution_x{};
    double resolution_y{};
  };

  struct CropBounds
  {
    int col_start{};
    int col_end{};
    int row_start{};
    int row_end{};
    double origin_x{};
    double origin_y{};
    double side_length_m{};
  };

  bool load_source_map()
  {
    const auto metadata_content = read_file(metadata_path_);
    if (!metadata_content.has_value()) {
      RCLCPP_ERROR(get_logger(), "Unable to read metadata json: %s", metadata_path_.c_str());
      return false;
    }

    const auto bbox = extract_bbox(*metadata_content);
    const auto abs_x = extract_scalar(*metadata_content, "abs_x");
    const auto abs_y = extract_scalar(*metadata_content, "abs_y");
    const auto lat0 = extract_scalar(*metadata_content, "lat0");
    const auto lon0 = extract_scalar(*metadata_content, "lon0");
    if (
      !bbox.has_value() || !abs_x.has_value() || !abs_y.has_value() ||
      !lat0.has_value() || !lon0.has_value())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Metadata json missing required fields: extent.bbox, resolution.abs_x/abs_y, origin.lat0/lon0");
      return false;
    }

    metadata_.min_x = (*bbox)[0];
    metadata_.min_y = (*bbox)[1];
    metadata_.max_x = (*bbox)[2];
    metadata_.max_y = (*bbox)[3];
    metadata_.resolution_x = std::abs(*abs_x);
    metadata_.resolution_y = std::abs(*abs_y);
    image_center_lat_ = *lat0;
    image_center_lon_ = *lon0;

    if (metadata_.resolution_x <= kEpsilon || metadata_.resolution_y <= kEpsilon) {
      RCLCPP_ERROR(get_logger(), "Metadata resolution must be positive");
      return false;
    }

    source_map_ = cv::imread(source_map_path_, cv::IMREAD_UNCHANGED);
    if (source_map_.empty()) {
      RCLCPP_ERROR(get_logger(), "Unable to read source image: %s", source_map_path_.c_str());
      return false;
    }

    if (source_map_.channels() != 1) {
      RCLCPP_ERROR(
        get_logger(),
        "Source map image must be single-channel grayscale, but got %d channels",
        source_map_.channels());
      return false;
    }

    if (source_map_.depth() != CV_8U) {
      RCLCPP_ERROR(
        get_logger(),
        "Source map image must be 8-bit grayscale, but got depth code %d",
        source_map_.depth());
      return false;
    }

    const auto expected_width = static_cast<double>(source_map_.cols) * metadata_.resolution_x;
    const auto expected_height = static_cast<double>(source_map_.rows) * metadata_.resolution_y;
    const auto metadata_width = metadata_.max_x - metadata_.min_x;
    const auto metadata_height = metadata_.max_y - metadata_.min_y;

    if (std::abs(expected_width - metadata_width) > 1e-3 ||
      std::abs(expected_height - metadata_height) > 1e-3)
    {
      RCLCPP_WARN(
        get_logger(),
        "Metadata/image extent mismatch (metadata: %.3fx%.3f m, image: %.3fx%.3f m)",
        metadata_width, metadata_height, expected_width, expected_height);
    }

    std::error_code ec;
    std::filesystem::create_directories(output_dir_, ec);
    if (ec) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to create output directory %s: %s",
        output_dir_.c_str(), ec.message().c_str());
      return false;
    }

    return true;
  }

  int next_crop_index() const
  {
    std::error_code ec;
    if (!std::filesystem::exists(output_dir_, ec)) {
      return 1;
    }

    int max_index = 0;
    for (const auto & entry : std::filesystem::directory_iterator(output_dir_)) {
      if (!entry.is_regular_file()) {
        continue;
      }

      const auto name = entry.path().filename().string();
      if (name.rfind(output_prefix_, 0) != 0U) {
        continue;
      }

      std::string suffix;
      if (ends_with(name, ".png")) {
        suffix = ".png";
      } else if (ends_with(name, ".yaml")) {
        suffix = ".yaml";
      } else {
        continue;
      }

      const auto serial = name.substr(output_prefix_.size(), name.size() - output_prefix_.size() - suffix.size());
      if (!is_digits_only(serial)) {
        continue;
      }

      try {
        max_index = std::max(max_index, std::stoi(serial));
      } catch (const std::exception &) {
        continue;
      }
    }

    return max_index + 1;
  }

  bool compute_crop_bounds(
    double current_x,
    double current_y,
    double goal_x,
    double goal_y,
    CropBounds * out_bounds,
    std::string * error_msg) const
  {
    const double distance_m = std::hypot(goal_x - current_x, goal_y - current_y);
    const double requested_side_m = distance_m + roi_padding_m_;

    const double map_width_m = metadata_.max_x - metadata_.min_x;
    const double map_height_m = metadata_.max_y - metadata_.min_y;
    const double max_square_side_m = std::min(map_width_m, map_height_m);

    if (max_square_side_m <= kEpsilon) {
      *error_msg = "Map metadata extent is invalid";
      return false;
    }

    double side_m = requested_side_m;
    if (side_m > max_square_side_m) {
      side_m = max_square_side_m;
      RCLCPP_WARN(
        get_logger(),
        "Requested ROI side %.3f m exceeds map extent %.3f m, clamping",
        requested_side_m, max_square_side_m);
    }

    const int side_cols = std::clamp(
      static_cast<int>(std::ceil(side_m / metadata_.resolution_x)),
      1, source_map_.cols);
    const int side_rows = std::clamp(
      static_cast<int>(std::ceil(side_m / metadata_.resolution_y)),
      1, source_map_.rows);

    const double center_x = 0.5 * (current_x + goal_x);
    const double center_y = 0.5 * (current_y + goal_y);
    const double center_col = (center_x - metadata_.min_x) / metadata_.resolution_x;
    const double center_row = (metadata_.max_y - center_y) / metadata_.resolution_y;

    int col_start = static_cast<int>(std::llround(center_col - static_cast<double>(side_cols) * 0.5));
    int row_start = static_cast<int>(std::llround(center_row - static_cast<double>(side_rows) * 0.5));

    col_start = std::clamp(col_start, 0, source_map_.cols - side_cols);
    row_start = std::clamp(row_start, 0, source_map_.rows - side_rows);

    const int col_end = col_start + side_cols;
    const int row_end = row_start + side_rows;

    if (col_end <= col_start || row_end <= row_start) {
      *error_msg = "Computed crop area is empty after clipping";
      return false;
    }

    // Origin is computed from snapped integer pixel bounds; no subpixel shift is possible.
    const double origin_x = metadata_.min_x + static_cast<double>(col_start) * metadata_.resolution_x;
    const double origin_y = metadata_.max_y - static_cast<double>(row_end) * metadata_.resolution_y;
    const double realized_side_x = static_cast<double>(col_end - col_start) * metadata_.resolution_x;
    const double realized_side_y = static_cast<double>(row_end - row_start) * metadata_.resolution_y;

    out_bounds->col_start = col_start;
    out_bounds->col_end = col_end;
    out_bounds->row_start = row_start;
    out_bounds->row_end = row_end;
    out_bounds->origin_x = origin_x;
    out_bounds->origin_y = origin_y;
    out_bounds->side_length_m = std::min(realized_side_x, realized_side_y);
    return true;
  }

  bool get_current_pose_in_map(double * out_x, double * out_y, std::string * error_msg)
  {
    try {
      const auto tf = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_s_));
      *out_x = tf.transform.translation.x;
      *out_y = tf.transform.translation.y;
      if (!std::isfinite(*out_x) || !std::isfinite(*out_y)) {
        *error_msg = "TF lookup returned non-finite current position";
        return false;
      }
      return true;
    } catch (const tf2::TransformException & ex) {
      *error_msg =
        "TF lookup failed (" + base_frame_ + " -> " + map_frame_ + "): " + std::string(ex.what());
      return false;
    }
  }

  bool convert_goal_wgs84_to_map(
    double latitude,
    double longitude,
    double * out_x,
    double * out_y,
    std::string * error_msg)
  {
    if (!std::isfinite(latitude) || !std::isfinite(longitude)) {
      *error_msg = "Goal latitude/longitude must be finite";
      return false;
    }

    if (!from_ll_client_->wait_for_service(std::chrono::duration<double>(from_ll_wait_timeout_s_))) {
      *error_msg = "fromLL service unavailable: " + from_ll_service_name_;
      return false;
    }

    auto request = std::make_shared<FromLL::Request>();
    request->ll_point.latitude = latitude;
    request->ll_point.longitude = longitude;
    request->ll_point.altitude = 0.0;

    auto future = from_ll_client_->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(from_ll_response_timeout_s_)) !=
      std::future_status::ready)
    {
      *error_msg = "Timeout waiting for fromLL response";
      return false;
    }

    const auto response = future.get();
    *out_x = response->map_point.x;
    *out_y = response->map_point.y;
    if (!std::isfinite(*out_x) || !std::isfinite(*out_y)) {
      *error_msg = "fromLL returned non-finite map coordinates";
      return false;
    }
    return true;
  }

  bool get_image_center_map_xy(double * out_x, double * out_y, std::string * error_msg)
  {
    if (image_center_map_cached_) {
      *out_x = image_center_map_x_;
      *out_y = image_center_map_y_;
      return true;
    }

    double center_x = 0.0;
    double center_y = 0.0;
    if (!convert_goal_wgs84_to_map(image_center_lat_, image_center_lon_, &center_x, &center_y, error_msg)) {
      *error_msg = "Failed to convert metadata center lat/lon via fromLL: " + *error_msg;
      return false;
    }

    image_center_map_x_ = center_x;
    image_center_map_y_ = center_y;
    image_center_map_cached_ = true;
    *out_x = center_x;
    *out_y = center_y;
    return true;
  }

  bool write_yaml(
    const std::filesystem::path & yaml_path,
    const std::string & image_name,
    double origin_x_map,
    double origin_y_map,
    std::string * error_msg) const
  {
    std::ofstream ofs(yaml_path);
    if (!ofs.is_open()) {
      *error_msg = "Failed to open output yaml: " + yaml_path.string();
      return false;
    }

    ofs << "# Auto-generated by map_cropper.\n";
    ofs << "image: \"" << image_name << "\"\n";
    ofs << std::setprecision(15);
    ofs << "resolution: " << metadata_.resolution_x << "\n";
    ofs << "origin: [" << origin_x_map << ", " << origin_y_map << ", 0.0]\n";
    ofs << "negate: " << yaml_negate_ << "\n";
    ofs << "occupied_thresh: " << yaml_occupied_thresh_ << "\n";
    ofs << "free_thresh: " << yaml_free_thresh_ << "\n";
    ofs << "mode: \"" << yaml_mode_ << "\"\n";

    if (!ofs.good()) {
      *error_msg = "Failed while writing yaml: " + yaml_path.string();
      return false;
    }

    return true;
  }

  bool call_load_map(const std::filesystem::path & yaml_path, uint8_t * result_code, std::string * error_msg)
  {
    if (!map_server_client_->wait_for_service(std::chrono::duration<double>(load_map_wait_timeout_s_))) {
      *error_msg = "map_server load_map service unavailable: " + map_server_service_name_;
      return false;
    }

    auto request = std::make_shared<LoadMap::Request>();
    request->map_url = yaml_path.string();

    auto future = map_server_client_->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(load_map_response_timeout_s_)) != std::future_status::ready) {
      *error_msg = "Timeout waiting for map_server/load_map response";
      return false;
    }

    const auto response = future.get();
    *result_code = response->result;

    if (response->result != LoadMap::Response::RESULT_SUCCESS) {
      std::ostringstream oss;
      oss << "map_server/load_map failed with code " << static_cast<int>(response->result);
      *error_msg = oss.str();
      return false;
    }

    return true;
  }

  void on_crop_request(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<CropMap::Request> request,
    std::shared_ptr<CropMap::Response> response)
  {
    response->success = false;
    response->load_map_result = LoadMap::Response::RESULT_UNDEFINED_FAILURE;
    response->current_map_x = std::numeric_limits<double>::quiet_NaN();
    response->current_map_y = std::numeric_limits<double>::quiet_NaN();
    response->goal_map_x = std::numeric_limits<double>::quiet_NaN();
    response->goal_map_y = std::numeric_limits<double>::quiet_NaN();

    if (!initialized_) {
      response->message = "map_cropper initialization failed; check logs";
      return;
    }

    double current_x = 0.0;
    double current_y = 0.0;
    double goal_x = 0.0;
    double goal_y = 0.0;
    double image_center_map_x = 0.0;
    double image_center_map_y = 0.0;

    std::string error_msg;
    if (!get_current_pose_in_map(&current_x, &current_y, &error_msg)) {
      response->message = error_msg;
      return;
    }
    response->current_map_x = current_x;
    response->current_map_y = current_y;

    if (!convert_goal_wgs84_to_map(
        request->goal_latitude, request->goal_longitude, &goal_x, &goal_y, &error_msg))
    {
      response->message = error_msg;
      return;
    }
    response->goal_map_x = goal_x;
    response->goal_map_y = goal_y;

    if (!get_image_center_map_xy(&image_center_map_x, &image_center_map_y, &error_msg)) {
      response->message = error_msg;
      return;
    }

    const double current_image_x = current_x - image_center_map_x;
    const double current_image_y = current_y - image_center_map_y;
    const double goal_image_x = goal_x - image_center_map_x;
    const double goal_image_y = goal_y - image_center_map_y;

    CropBounds bounds;
    if (!compute_crop_bounds(
        current_image_x, current_image_y, goal_image_x, goal_image_y, &bounds, &error_msg))
    {
      response->message = error_msg;
      return;
    }

    const double origin_map_x = bounds.origin_x + image_center_map_x;
    const double origin_map_y = bounds.origin_y + image_center_map_y;

    const int crop_width = bounds.col_end - bounds.col_start;
    const int crop_height = bounds.row_end - bounds.row_start;

    const auto crop_rect = cv::Rect(bounds.col_start, bounds.row_start, crop_width, crop_height);
    const cv::Mat crop = source_map_(crop_rect).clone();
    if (crop.empty()) {
      response->message = "Failed to extract crop image";
      return;
    }

    const int crop_index = next_crop_index();
    const std::string stem = output_prefix_ + std::to_string(crop_index);

    const auto output_png = std::filesystem::path(output_dir_) / (stem + ".png");
    const auto output_yaml = std::filesystem::path(output_dir_) / (stem + ".yaml");

    if (!cv::imwrite(output_png.string(), crop)) {
      response->message = "Failed to write crop image: " + output_png.string();
      return;
    }

    if (!write_yaml(output_yaml, output_png.filename().string(), origin_map_x, origin_map_y, &error_msg)) {
      response->message = error_msg;
      return;
    }

    uint8_t load_result = LoadMap::Response::RESULT_UNDEFINED_FAILURE;
    if (!call_load_map(output_yaml, &load_result, &error_msg)) {
      response->load_map_result = load_result;
      response->crop_image_path = output_png.string();
      response->crop_yaml_path = output_yaml.string();
      response->roi_bottom_left_x = origin_map_x;
      response->roi_bottom_left_y = origin_map_y;
      response->roi_side_length_m = bounds.side_length_m;
      response->map_origin_x = origin_map_x;
      response->map_origin_y = origin_map_y;
      response->message = error_msg;
      return;
    }

    response->success = true;
    response->load_map_result = load_result;
    response->crop_image_path = output_png.string();
    response->crop_yaml_path = output_yaml.string();
    response->roi_bottom_left_x = origin_map_x;
    response->roi_bottom_left_y = origin_map_y;
    response->roi_side_length_m = bounds.side_length_m;
    response->map_origin_x = origin_map_x;
    response->map_origin_y = origin_map_y;
    response->message = "Crop generated and loaded";

    RCLCPP_INFO(
      get_logger(),
      "Generated %s and loaded %s (current_map=[%.3f, %.3f], goal_map=[%.3f, %.3f], image_center_map=[%.3f, %.3f], origin_map=[%.3f, %.3f], side=%.3f m)",
      output_png.c_str(), output_yaml.c_str(), current_x, current_y, goal_x, goal_y,
      image_center_map_x, image_center_map_y, origin_map_x, origin_map_y, bounds.side_length_m);
  }

  bool initialized_{false};

  std::string source_map_path_;
  std::string metadata_path_;
  std::string output_dir_;
  std::string output_prefix_;
  std::string service_name_;
  std::string map_server_service_name_;
  std::string from_ll_service_name_;
  std::string map_frame_;
  std::string base_frame_;
  double image_center_lat_{0.0};
  double image_center_lon_{0.0};
  bool image_center_map_cached_{false};
  double image_center_map_x_{0.0};
  double image_center_map_y_{0.0};

  double roi_padding_m_{100.0};
  double tf_timeout_s_{0.2};
  double from_ll_wait_timeout_s_{2.0};
  double from_ll_response_timeout_s_{5.0};
  double load_map_wait_timeout_s_{3.0};
  double load_map_response_timeout_s_{10.0};

  std::string yaml_mode_;
  int yaml_negate_{1};
  double yaml_occupied_thresh_{1.0};
  double yaml_free_thresh_{0.0};

  MapMetadata metadata_;
  cv::Mat source_map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr client_group_;
  rclcpp::Service<CropMap>::SharedPtr crop_service_;
  rclcpp::Client<LoadMap>::SharedPtr map_server_client_;
  rclcpp::Client<FromLL>::SharedPtr from_ll_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MapCropper>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
