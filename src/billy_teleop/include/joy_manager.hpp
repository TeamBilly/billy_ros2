#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "billy_description/billy_description.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

#define VERBOSE 0

class JoyManager : public rclcpp::Node
{
  public:
    JoyManager();

  private:
    billy_description::BillyDescription BILLY_CONST;
    void createPublishers();
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_cmd_vel_;
    void callbackTimerCmdVel();

    void createSubscribers();
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_joy_;
    void callbackJoy(sensor_msgs::msg::Joy::SharedPtr msg);

    // Variables
    float joy_turn_limited_;
    std::chrono::steady_clock::time_point rate_limiter_previous_time_turn_;
    float joy_speed_limited_;
    std::chrono::steady_clock::time_point rate_limiter_previous_time_speed_;

    template <typename T> T rateLimiter(T input, 
                      T output_previous, 
                      std::chrono::steady_clock::time_point &previous_time);

    size_t count_;


};
