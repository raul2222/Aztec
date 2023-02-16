#include <memory>
#include "cs20_node.cpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rplidar_ros::rplidar_node>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}