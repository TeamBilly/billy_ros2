#include "joy_manager.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyManager>());
  rclcpp::shutdown();
  return 0;
}
