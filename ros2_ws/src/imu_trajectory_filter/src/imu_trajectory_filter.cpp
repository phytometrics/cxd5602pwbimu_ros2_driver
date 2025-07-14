#include "imu_trajectory_filter/imu_trajectory_filter.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

ImuTrajectoryFilter::ImuTrajectoryFilter(const rclcpp::NodeOptions & options)
: Node("imu_trajectory_filter", options),
  first_msg_(true)
{
  this->declare_parameter("alpha", 0.98);
  this->declare_parameter("base_frame", "imu_link");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("max_time_delta", 2.0);
  this->declare_parameter("warn_time_delta", 0.1);
  
  alpha_ = this->get_parameter("alpha").as_double();
  base_frame_ = this->get_parameter("base_frame").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  max_time_delta_ = this->get_parameter("max_time_delta").as_double();
  warn_time_delta_ = this->get_parameter("warn_time_delta").as_double();
  
  RCLCPP_INFO(this->get_logger(), "Filter initialized with alpha=%.2f, max_dt=%.2f, warn_dt=%.2f", 
             alpha_, max_time_delta_, warn_time_delta_);
  
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_raw", 10,
    std::bind(&ImuTrajectoryFilter::imuCallback, this, std::placeholders::_1));
  
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/imu/pose", 10);
  
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/imu/trajectory", 10);
  
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  
  trajectory_.header.frame_id = odom_frame_;
  
  velocity_.x = velocity_.y = velocity_.z = 0.0;
  position_.x = position_.y = position_.z = 0.0;
  prev_linear_accel_.x = prev_linear_accel_.y = prev_linear_accel_.z = 0.0;
  
  current_pose_.header.frame_id = odom_frame_;
  current_pose_.pose.position.x = 0.0;
  current_pose_.pose.position.y = 0.0;
  current_pose_.pose.position.z = 0.0;
  current_pose_.pose.orientation.w = 1.0;
  current_pose_.pose.orientation.x = 0.0;
  current_pose_.pose.orientation.y = 0.0;
  current_pose_.pose.orientation.z = 0.0;
  
  RCLCPP_INFO(this->get_logger(), "IMU Trajectory Filter node started");
}

void ImuTrajectoryFilter::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  if (first_msg_) {
    first_msg_ = false;
    last_time_ = msg->header.stamp;
    return;
  }
  
  rclcpp::Time current_time = msg->header.stamp;
  dt_ = (current_time - last_time_).seconds();
  
  // More flexible timestamp validation
  if (dt_ <= 0.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "Non-progressive timestamp: dt=%.6f", dt_);
    last_time_ = current_time;
    return;
  }
  
  if (dt_ > max_time_delta_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "Large time gap: dt=%.6f, resetting filter", dt_);
    // Reset filter state for large gaps
    velocity_.x = velocity_.y = velocity_.z = 0.0;
    last_time_ = current_time;
    return;
  }
  
  if (dt_ > warn_time_delta_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                        "Unusually large time delta: %.6f", dt_);
  }
  
  // Apply complementary filter to linear acceleration
  geometry_msgs::msg::Vector3 filtered_accel;
  filtered_accel.x = alpha_ * prev_linear_accel_.x + (1.0 - alpha_) * msg->linear_acceleration.x;
  filtered_accel.y = alpha_ * prev_linear_accel_.y + (1.0 - alpha_) * msg->linear_acceleration.y;
  filtered_accel.z = alpha_ * prev_linear_accel_.z + (1.0 - alpha_) * msg->linear_acceleration.z;
  
  // Remove gravity (assuming Z is up)
  filtered_accel.z -= 9.81;
  
  // Integrate acceleration to get velocity
  velocity_.x += filtered_accel.x * dt_;
  velocity_.y += filtered_accel.y * dt_;
  velocity_.z += filtered_accel.z * dt_;
  
  // Integrate velocity to get position
  position_.x += velocity_.x * dt_;
  position_.y += velocity_.y * dt_;
  position_.z += velocity_.z * dt_;
  
  // Update current pose
  current_pose_.header.stamp = current_time;
  current_pose_.pose.position.x = position_.x;
  current_pose_.pose.position.y = position_.y;
  current_pose_.pose.position.z = position_.z;
  current_pose_.pose.orientation = msg->orientation;
  
  // Publish pose
  pose_pub_->publish(current_pose_);
  
  // Add to trajectory
  trajectory_.poses.push_back(current_pose_);
  trajectory_.header.stamp = current_time;
  
  // Limit trajectory size
  if (trajectory_.poses.size() > 1000) {
    trajectory_.poses.erase(trajectory_.poses.begin());
  }
  
  // Publish trajectory
  path_pub_->publish(trajectory_);
  
  // Publish transform
  publishTransform(current_pose_);
  
  // Update for next iteration
  prev_linear_accel_ = filtered_accel;
  last_time_ = current_time;
}

void ImuTrajectoryFilter::publishTransform(const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = pose.header.stamp;
  transform.header.frame_id = odom_frame_;
  transform.child_frame_id = base_frame_;
  
  transform.transform.translation.x = pose.pose.position.x;
  transform.transform.translation.y = pose.pose.position.y;
  transform.transform.translation.z = pose.pose.position.z;
  transform.transform.rotation = pose.pose.orientation;
  
  tf_broadcaster_->sendTransform(transform);
}