#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "billy_description/billy_description.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class JoyManager : public rclcpp::Node
{
  public:
    JoyManager();

  private:
    // Init
    billy_description::BillyDescription BILLY_CONST;
    void initParam();
    uint8_t p_verbose_;
    float p_deadzone_;
    float p_straight_speed_;
    float p_half_turn_speed_;
    uint16_t p_callback_cmd_vel_period_;
    uint16_t p_callback_mode_period_;


    // Publishers
    void createPublishers();
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr publisher_mode_;
    rclcpp::TimerBase::SharedPtr timer_mode_;
    void callbackTimerMode();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_cmd_vel_;
    void callbackTimerCmdVel();


    // Subscribers
    void createSubscribers();
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriber_joy_;
    void callbackJoy(sensor_msgs::msg::Joy::SharedPtr msg);

    // Variables
    void applyMode();

    float joy_turn_;
    float joy_turn_limited_;
    float joy_turn_selected_;
    std::chrono::steady_clock::time_point rate_limiter_previous_time_turn_;
    float joy_speed_;
    float joy_speed_limited_;
    float joy_speed_selected_;
    std::chrono::steady_clock::time_point rate_limiter_previous_time_speed_;

    template <typename T> T rateLimiter(T input, 
                      T output_previous, 
                      std::chrono::steady_clock::time_point &previous_time);

    uint8_t current_mode_;
    bool rising_edge_up_arrow_;
    bool rising_edge_down_arrow_;
    bool rising_edge_left_arrow_;
    bool rising_edge_right_arrow_;

    size_t count_;


};
