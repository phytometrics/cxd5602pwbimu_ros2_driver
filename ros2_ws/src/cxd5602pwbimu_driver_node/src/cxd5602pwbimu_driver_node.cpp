/*
* Copyright (c) 2025 NITK.K ROS-Team
*
* SPDX-License-Identifier: Apache-2.0
*/

#include "cxd5602pwbimu_driver_node/cxd5602pwbimu_driver_node.hpp"

namespace cxd5602pwbimu_driver_node
{

Cxd5602pwbimuDriverNode::Cxd5602pwbimuDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("imu_publisher", options),
  port_handler_(this->declare_parameter<std::string>("dev", "/dev/ttyUSB0")),
  time_offset_(0),
  delimiter_(this->declare_parameter<char>("delimiter", '\n'))
{
  const int baudrate = this->declare_parameter<int>("baudrate", 115200);
  const int timeout_ms = this->declare_parameter<int>("timeout_ms", 100);
  const int spin_ms = this->declare_parameter<int>("spin_ms", 1);

  imu_ = std::make_unique<ImuClass>();
  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "/imu/data_raw", rclcpp::SensorDataQoS().reliable());

  if (!this->port_handler_.configure(baudrate, timeout_ms)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure serial port");
    exit(EXIT_FAILURE);
  }

  if (!this->port_handler_.open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
    exit(EXIT_FAILURE);
  }

  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(spin_ms), std::bind(&Cxd5602pwbimuDriverNode::timerCallback, this));
}

Cxd5602pwbimuDriverNode::~Cxd5602pwbimuDriverNode()
{
  this->port_handler_.close();
}

void Cxd5602pwbimuDriverNode::timerCallback()
{
  this->timer_->cancel();

  std::stringstream ss;
  this->port_handler_.readUntil(ss, delimiter_);

  std::string buffer = ss.str();

  if (imu_->set_data(reinterpret_cast<const uint8_t *>(buffer.c_str()), buffer.size())) {
    auto [linear_acceleration, angular_velocity, sec, msec] = imu_->get_data();

    // Validate timestamp
    if (sec == 0 && msec == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid timestamp from firmware");
      this->timer_->reset();
      return;
    }

    auto msg = std::make_unique<sensor_msgs::msg::Imu>();

    if (time_offset_ == 0) {
      time_offset_ = static_cast<uint32_t>(std::time(nullptr));
      RCLCPP_INFO(this->get_logger(), "Time offset initialized: %u", time_offset_);
    }

    msg->header.frame_id = "imu";
    msg->header.stamp.sec = sec + time_offset_;
    msg->header.stamp.nanosec = msec * 1000000;

    // Validate timestamp progression
    static rclcpp::Time last_timestamp;
    rclcpp::Time current_timestamp = msg->header.stamp;
    
    if (last_timestamp.nanoseconds() > 0) {
      double dt = (current_timestamp - last_timestamp).seconds();
      if (dt <= 0.0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "Non-progressive timestamp detected: dt=%.6f", dt);
      } else if (dt > 0.1) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "Large timestamp gap detected: dt=%.6f", dt);
      }
    }
    last_timestamp = current_timestamp;

    msg->linear_acceleration.x = linear_acceleration[0];
    msg->linear_acceleration.y = linear_acceleration[1];
    msg->linear_acceleration.z = linear_acceleration[2];

    msg->angular_velocity.x = angular_velocity[0] * 0.5;
    msg->angular_velocity.y = angular_velocity[1] * 0.5;
    msg->angular_velocity.z = angular_velocity[2] * 0.5;

    // Set covariance matrices to indicate unknown covariance
    for (int i = 0; i < 9; i++) {
      msg->linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
      msg->angular_velocity_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
      msg->orientation_covariance[i] = (i % 4 == 0) ? -1.0 : 0.0; // -1 means unknown
    }

    publisher_->publish(std::move(msg));
  }

  this->timer_->reset();
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cxd5602pwbimu_driver_node::Cxd5602pwbimuDriverNode)
