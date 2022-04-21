#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

#define RISING_SLEW_RATE 1
#define FALLING_SLEW_RATE -1
#define SPEED_MAX 10
#define TURN_MAX 30
#define VERBOSE 0

class JoyManager : public rclcpp::Node
{
  public:
    JoyManager();

  private:
    void createPublishers();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    void timer_callback();

    void createSubscribers();
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_joy_;
    void callbackJoy(sensor_msgs::msg::Joy::SharedPtr msg);

    // Variables
    float joy_turn_;
    float joy_speed_;
    float joy_speed_previous_;

    float rateLimiter(float input, float output_previous);
    std::chrono::steady_clock::time_point rate_limiter_previous_time_;

    size_t count_;


};
