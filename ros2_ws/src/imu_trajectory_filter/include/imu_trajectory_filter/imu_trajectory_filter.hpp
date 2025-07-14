#ifndef IMU_TRAJECTORY_FILTER__IMU_TRAJECTORY_FILTER_HPP_
#define IMU_TRAJECTORY_FILTER__IMU_TRAJECTORY_FILTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ImuTrajectoryFilter : public rclcpp::Node
{
public:
  explicit ImuTrajectoryFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void publishTrajectory();
  void publishTransform(const geometry_msgs::msg::PoseStamped & pose);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  nav_msgs::msg::Path trajectory_;
  geometry_msgs::msg::PoseStamped current_pose_;
  
  // Filter parameters
  double alpha_;
  double dt_;
  double max_time_delta_;
  double warn_time_delta_;
  
  // State variables
  geometry_msgs::msg::Vector3 velocity_;
  geometry_msgs::msg::Vector3 position_;
  geometry_msgs::msg::Vector3 prev_linear_accel_;
  
  // Frame IDs
  std::string base_frame_;
  std::string odom_frame_;
  
  bool first_msg_;
  rclcpp::Time last_time_;
};

#endif  // IMU_TRAJECTORY_FILTER__IMU_TRAJECTORY_FILTER_HPP_