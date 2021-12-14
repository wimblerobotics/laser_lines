#include "laser_lines/line_extraction_ros.h"

#include <cmath>

#include "point_line_util.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

namespace laser_lines {

///////////////////////////////////////////////////////////////////////////////
// Constructor / destructor
///////////////////////////////////////////////////////////////////////////////
LineExtractionROS::LineExtractionROS(rclcpp::Node::SharedPtr nh)
    : nh_(nh), data_cached_(false) {
  loadParameters();

  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  qos.avoid_ros_namespace_conventions(false);

  line_publisher_ = nh_->create_publisher<laser_lines::msg::LineSegmentList>(
      "line_segments", qos);
  closest_point_to_line_publisher_ =
      nh_->create_publisher<laser_lines::msg::ClosestPointToLineSegmentList>(
          "closest_point_to_line_segments", qos);
  scan_subscriber_ = nh_->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, qos,
      std::bind(&LineExtractionROS::laserScanCallback, this, _1));
  if (publish_markers_) {
    marker_publisher_ = nh_->create_publisher<visualization_msgs::msg::Marker>(
        "line_markers", qos);
  }

  callback_handle_ = nh->add_on_set_parameters_callback(std::bind(
      &LineExtractionROS::parametersCallback, this, std::placeholders::_1));
}

LineExtractionROS::~LineExtractionROS() {}

///////////////////////////////////////////////////////////////////////////////
// Run
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::run() {
  // Extract the lines
  // std::vector<Line> lines;
  // line_extraction_.extractLines(lines);

  // // Populate message
  // laser_lines::msg::LineSegmentList msg;
  // populateLineSegListMsg(lines, msg);

  // // Publish the lines
  // line_publisher_->publish(msg);

  // // Also publish markers if parameter publish_markers is set to true
  // if (publish_markers_) {
  //   visualization_msgs::msg::Marker marker_msg;
  //   populateMarkerMsg(lines, marker_msg);
  //   marker_publisher_->publish(marker_msg);
  // }
}

rcl_interfaces::msg::SetParametersResult LineExtractionROS::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &parameter : parameters) {
    if ((parameter.get_name() == "bearing_std_dev") &&
        (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)) {
      bearing_std_dev_ = parameter.as_double();
      line_extraction_.setBearingVariance(bearing_std_dev_ * bearing_std_dev_);
      RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                  "Parameter 'bearing_std_dev' changed: %f", bearing_std_dev_);
    } else if ((parameter.get_name() == "frame_id") &&
               (parameter.get_type() ==
                rclcpp::ParameterType::PARAMETER_STRING)) {
      frame_id_ = parameter.as_string();
      RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                  "Parameter 'frame_id' changed: %s", frame_id_.c_str());
    } else if ((parameter.get_name() == "least_sq_angle_thresh") &&
               (parameter.get_type() ==
                rclcpp::ParameterType::PARAMETER_DOUBLE)) {
      least_sq_angle_thresh_ = parameter.as_double();
      line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh_);
      RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                  "Parameter 'least_sq_angle_thresh' changed: %f",
                  least_sq_angle_thresh_);
    } else if ((parameter.get_name() == "least_sq_radius_thresh") &&
               (parameter.get_type() ==
                rclcpp::ParameterType::PARAMETER_DOUBLE)) {
      least_sq_radius_thresh_ = parameter.as_double();
      line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh_);
      RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                  "Parameter 'least_sq_radius_thresh' changed: %f",
                  least_sq_radius_thresh_);
    } else if ((parameter.get_name() == "max_line_gap") &&
               (parameter.get_type() ==
                rclcpp::ParameterType::PARAMETER_DOUBLE)) {
      max_line_gap_ = parameter.as_double();
      line_extraction_.setMaxLineGap(max_line_gap_);
      RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                  "Parameter 'max_line_gap' changed: %f", max_line_gap_);
    } else if ((parameter.get_name() == "max_range") &&
               (parameter.get_type() ==
                rclcpp::ParameterType::PARAMETER_DOUBLE)) {
      max_range_ = parameter.as_double();
      line_extraction_.setMaxRange(max_range_);
      RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                  "Parameter 'max_range' changed: %f", max_range_);
    } else if ((parameter.get_name() == "min_line_length") &&
               (parameter.get_type() ==
                rclcpp::ParameterType::PARAMETER_DOUBLE)) {
      min_line_length_ = parameter.as_double();
      line_extraction_.setMinLineLength(min_line_length_);
      RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                  "Parameter 'min_line_length' changed: %f", min_line_length_);
    } else if ((parameter.get_name() == "min_line_points") &&
               (parameter.get_type() ==
                rclcpp::ParameterType::PARAMETER_INTEGER)) {
      min_line_points_ = parameter.as_int();
      line_extraction_.setMinLinePoints(
          static_cast<unsigned int>(min_line_points_));
      RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                  "Parameter 'min_line_points' changed: %d", min_line_points_);
    } else if ((parameter.get_name() == "min_range") &&
               (parameter.get_type() ==
                rclcpp::ParameterType::PARAMETER_DOUBLE)) {
      min_range_ = parameter.as_double();
      line_extraction_.setMinRange(min_range_);
      RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                  "Parameter 'min_range' changed: %f", min_range_);
    } else if ((parameter.get_name() == "min_split_dist") &&
               (parameter.get_type() ==
                rclcpp::ParameterType::PARAMETER_DOUBLE)) {
      min_split_dist_ = parameter.as_double();
      line_extraction_.setMinSplitDist(min_split_dist_);
      RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                  "Parameter 'min_split_dist' changed: %f", min_split_dist_);
    } else if ((parameter.get_name() == "outlier_dist") &&
               (parameter.get_type() ==
                rclcpp::ParameterType::PARAMETER_DOUBLE)) {
      outlier_dist_ = parameter.as_double();
      line_extraction_.setOutlierDist(outlier_dist_);
      RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                  "Parameter 'outlier_dist' changed: %f", outlier_dist_);
    } else if ((parameter.get_name() == "publish_markers") &&
               (parameter.get_type() ==
                rclcpp::ParameterType::PARAMETER_BOOL)) {
      publish_markers_ = parameter.as_bool();
      RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                  "Parameter 'publish_markers' changed: %d", publish_markers_);
    } else if ((parameter.get_name() == "range_std_dev") &&
               (parameter.get_type() ==
                rclcpp::ParameterType::PARAMETER_DOUBLE)) {
      range_std_dev_ = parameter.as_double();
      line_extraction_.setRangeVariance(range_std_dev_ * range_std_dev_);
      RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                  "Parameter 'range_std_dev' changed: %f", range_std_dev_);
    } else if ((parameter.get_name() == "scan_topic") &&
               (parameter.get_type() ==
                rclcpp::ParameterType::PARAMETER_STRING)) {
      scan_topic_ = parameter.as_string();
      RCLCPP_INFO(rclcpp::get_logger("LineExtractionROS"),
                  "Parameter 'scan_topic' changed: %s", scan_topic_.c_str());
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("LineExtractionROS"),
                   "Invalid parameter: %s", parameter.get_name().c_str());
    }
  }

  return result;
}

///////////////////////////////////////////////////////////////////////////////
// Load ROS parameters
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::loadParameters() {
  nh_->declare_parameter("bearing_std_dev", 1e-3);
  nh_->declare_parameter("frame_id", "base_link");
  nh_->declare_parameter("least_sq_angle_thresh", 1e-4);
  nh_->declare_parameter("least_sq_radius_thresh", 1e-4);
  nh_->declare_parameter("max_line_gap", 0.4);
  nh_->declare_parameter("max_range", 20.0);
  nh_->declare_parameter("min_line_length", 0.5);
  nh_->declare_parameter("min_line_points", 9);
  nh_->declare_parameter("min_range", 0.4);
  nh_->declare_parameter("min_split_dist", 0.05);
  nh_->declare_parameter("outlier_dist", 0.05);
  nh_->declare_parameter("publish_markers", true);
  nh_->declare_parameter("range_std_dev", 0.02);
  nh_->declare_parameter("scan_topic", "scan");

  bearing_std_dev_ = nh_->get_parameter("bearing_std_dev").as_double();
  line_extraction_.setBearingVariance(bearing_std_dev_ * bearing_std_dev_);

  frame_id_ = nh_->get_parameter("frame_id").as_string();

  least_sq_angle_thresh_ =
      nh_->get_parameter("least_sq_angle_thresh").as_double();
  line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh_);

  least_sq_radius_thresh_ =
      nh_->get_parameter("least_sq_radius_thresh").as_double();
  line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh_);

  max_line_gap_ = nh_->get_parameter("max_line_gap").as_double();
  line_extraction_.setMaxLineGap(max_line_gap_);

  max_range_ = nh_->get_parameter("max_range").as_double();
  line_extraction_.setMaxRange(max_range_);

  min_line_length_ = nh_->get_parameter("min_line_length").as_double();
  line_extraction_.setMinLineLength(min_line_length_);

  min_line_points_ = nh_->get_parameter("min_line_points").as_int();
  line_extraction_.setMinLinePoints(
      static_cast<unsigned int>(min_line_points_));

  min_range_ = nh_->get_parameter("min_range").as_double();
  line_extraction_.setMinRange(min_range_);

  min_split_dist_ = nh_->get_parameter("min_split_dist").as_double();
  line_extraction_.setMinSplitDist(min_split_dist_);

  outlier_dist_ = nh_->get_parameter("outlier_dist").as_double();
  line_extraction_.setOutlierDist(outlier_dist_);

  publish_markers_ = nh_->get_parameter("publish_markers").as_bool();

  range_std_dev_ = nh_->get_parameter("range_std_dev").as_double();
  line_extraction_.setRangeVariance(range_std_dev_ * range_std_dev_);

  scan_topic_ = nh_->get_parameter("scan_topic").as_string();
}

///////////////////////////////////////////////////////////////////////////////
// Populate messages
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::populateClosetPointToLineSegListMsg(
    const std::vector<Line> &lines,
    laser_lines::msg::ClosestPointToLineSegmentList &line_list_msg) {
  for (std::vector<Line>::const_iterator cit = lines.begin();
       cit != lines.end(); ++cit) {
    laser_lines::msg::ClosestPointToLineSegment line_msg;
    // line_msg.covariance = cit->getCovariance();
    float qX;
    float qY;
    float d = DistanceFromLineSegmentToPoint(
        cit->getStart()[0], cit->getStart()[1], cit->getEnd()[0],
        cit->getEnd()[1], 0.0, 0.0, &qX, &qY);
    line_msg.start[0] = 0.0;
    line_msg.start[1] = 0.0;
    line_msg.end[0] = qX;
    line_msg.end[1] = qY;
    line_msg.distance = d;

    line_list_msg.closest_point_to_line_segments.push_back(line_msg);
  }

  line_list_msg.header.frame_id = frame_id_;
  line_list_msg.header.stamp = nh_->now();
}

void LineExtractionROS::populateLineSegListMsg(
    const std::vector<Line> &lines,
    laser_lines::msg::LineSegmentList &line_list_msg) {
  for (std::vector<Line>::const_iterator cit = lines.begin();
       cit != lines.end(); ++cit) {
    laser_lines::msg::LineSegment line_msg;
    line_msg.angle = cit->getAngle();
    line_msg.radius = cit->getRadius();
    auto covariance = cit->getCovariance();
    for (std::size_t i = 0; i < covariance.size(); i++) {
      line_msg.covariance[(int)i] = covariance[(int)i];
    }
    // line_msg.covariance = cit->getCovariance();
    auto start = cit->getStart();
    for (std::size_t i = 0; i < start.size(); i++) {
      line_msg.start[(int)i] = start[(int)i];
    }
    // line_msg.start = cit->getStart();
    auto end = cit->getEnd();
    for (std::size_t i = 0; i < end.size(); i++) {
      line_msg.end[(int)i] = end[(int)i];
    }
    // line_msg.end = cit->getEnd();
    line_list_msg.line_segments.push_back(line_msg);
  }

  line_list_msg.header.frame_id = frame_id_;
  line_list_msg.header.stamp = nh_->now();
}

void LineExtractionROS::populateMarkerMsg(
    const std::vector<Line> &lines,
    visualization_msgs::msg::Marker &marker_msg) {
  marker_msg.ns = "laser_lines";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_msg.scale.x = 0.1;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;
  for (std::vector<Line>::const_iterator cit = lines.begin();
       cit != lines.end(); ++cit) {
    geometry_msgs::msg::Point p_start;
    p_start.x = cit->getStart()[0];
    p_start.y = cit->getStart()[1];
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
    geometry_msgs::msg::Point p_end;
    p_end.x = cit->getEnd()[0];
    p_end.y = cit->getEnd()[1];
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }
  marker_msg.header.frame_id = frame_id_;
  marker_msg.header.stamp = nh_->now();
}

void LineExtractionROS::populateIntersectionMarkers(
    const std::vector<Line> &lines,
    visualization_msgs::msg::Marker &marker_msg) {
  marker_msg.ns = "closest_point_to_line_segments";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker_msg.scale.x = 0.1;
  marker_msg.color.r = 0.0;
  marker_msg.color.g = 1.0;
  marker_msg.color.b = 1.0;
  marker_msg.color.a = 1.0;
  for (std::vector<Line>::const_iterator cit = lines.begin();
       cit != lines.end(); ++cit) {
    float qX;
    float qY;
    float d = DistanceFromLineSegmentToPoint(
        cit->getStart()[0], cit->getStart()[1], cit->getEnd()[0],
        cit->getEnd()[1], 0.0, 0.0, &qX, &qY);
    geometry_msgs::msg::Point p_start;
    p_start.x = 0;
    p_start.y = 0;
    p_start.z = 0;
    marker_msg.points.push_back(p_start);
    geometry_msgs::msg::Point p_end;
    p_end.x = qX;
    p_end.y = qY;
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }
  marker_msg.header.frame_id = frame_id_;
  marker_msg.header.stamp = nh_->now();
}

///////////////////////////////////////////////////////////////////////////////
// Cache data on first LaserScan message received
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::cacheData(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;
  const std::size_t num_measurements = std::ceil(
      (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
  for (std::size_t i = 0; i < num_measurements; ++i) {
    const double b = scan_msg->angle_min + i * scan_msg->angle_increment;
    bearings.push_back(b);
    cos_bearings.push_back(cos(b));
    sin_bearings.push_back(sin(b));
    indices.push_back(i);
  }

  line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
  RCUTILS_LOG_DEBUG("Data has been cached.");
}

///////////////////////////////////////////////////////////////////////////////
// Main LaserScan callback
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::laserScanCallback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
  if (!data_cached_) {
    cacheData(scan_msg);
    data_cached_ = true;
  }

  std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(),
                                          scan_msg->ranges.end());
  line_extraction_.setRangeData(scan_ranges_doubles);
  std::vector<Line> lines;
  line_extraction_.extractLines(lines);

  // Populate ListSegmentList message
  laser_lines::msg::LineSegmentList msg;
  populateLineSegListMsg(lines, msg);

  // Publish the lines
  line_publisher_->publish(msg);

  // Populate ClosestPointToListSegmentList message
  laser_lines::msg::ClosestPointToLineSegmentList msg2;
  populateClosetPointToLineSegListMsg(lines, msg2);

  // Publish the lines
  closest_point_to_line_publisher_->publish(msg2);

  // Also publish markers if parameter publish_markers is set to true
  if (publish_markers_) {
    visualization_msgs::msg::Marker marker_msg;
    populateMarkerMsg(lines, marker_msg);
    marker_msg.header.stamp = scan_msg->header.stamp;
    marker_publisher_->publish(marker_msg);
    visualization_msgs::msg::Marker marker_msg2;
    populateIntersectionMarkers(lines, marker_msg2);
    marker_msg2.header.stamp = scan_msg->header.stamp;
    marker_publisher_->publish(marker_msg2);
  }
}

}  // namespace laser_lines
