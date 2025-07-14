#include "imu_trajectory_filter/imu_trajectory_filter.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ImuTrajectoryFilter>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}