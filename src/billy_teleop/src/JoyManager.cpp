#include "joy_manager.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

JoyManager::JoyManager() : Node("joy_manager_node"), count_(0)
{
    initParam();
    createPublishers();
    createSubscribers();
}

void JoyManager::initParam()
{
    // Parameters verbose. Debug purposes.
    this->declare_parameter<int>("verbose", 0);
    this->get_parameter("verbose", p_verbose_);
    RCLCPP_INFO(this->get_logger(), "Init param 'verbose': '%d'", p_verbose_);

    // Parameters callback_cmd_vel_period. Define the /cmd_vel publish frequecy in ms.
    this->declare_parameter<int>("callback_cmd_vel_period", 10);
    this->get_parameter("callback_cmd_vel_period", p_callback_cmd_vel_period_);
    RCLCPP_INFO(this->get_logger(), "Init param 'callback_cmd_vel_period': '%d'",
      p_callback_cmd_vel_period_);

}

void JoyManager::createPublishers()
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&JoyManager::timer_callback, this));

    publisher_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto period_cmd_vel = p_callback_cmd_vel_period_ * 1ms;
    timer_cmd_vel_ = this->create_wall_timer(period_cmd_vel, std::bind(&JoyManager::callbackTimerCmdVel, this));
}

void JoyManager::createSubscribers()
{
    subscriber_joy_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoyManager::callbackJoy, this, _1));
}

void JoyManager::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

void JoyManager::callbackTimerCmdVel()
{
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = joy_speed_limited_;
    message.angular.z = joy_turn_limited_;
    publisher_cmd_vel_->publish(message);
}

void JoyManager::callbackJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Fetching info
    auto joy_speed = msg->axes[1];
    auto joy_turn = msg->axes[3];

    // Adjust to config values
    joy_speed *= BILLY_CONST.MOTION.LIN_X_MAX;
    joy_turn *= BILLY_CONST.MOTION.ANG_Z_MAX;

    // Limit rate of values
    joy_speed_limited_ = rateLimiter(joy_speed, joy_speed_limited_, rate_limiter_previous_time_speed_);
    joy_turn_limited_ = rateLimiter(joy_turn, joy_turn_limited_, rate_limiter_previous_time_turn_);
}

template <typename T> T JoyManager::rateLimiter(T input, 
                              T output_previous, 
                              std::chrono::steady_clock::time_point &previous_time)
{
    // Computing time difference
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_diff = 
      std::chrono::duration_cast<std::chrono::duration<double>>(now - previous_time);


    // Compute the rate
    T rate = (input - output_previous) / time_diff.count();

    // Evaluating rate
    T output = output_previous;
    if(rate > BILLY_CONST.MOTION.ACCEL_MAX) {
        output = time_diff.count() * BILLY_CONST.MOTION.ACCEL_MAX + output_previous;
    } else if (rate < BILLY_CONST.MOTION.DECEL_MAX) {
        output = time_diff.count() * BILLY_CONST.MOTION.DECEL_MAX + output_previous;
    } else {
        output = input;
    }

    // Debug
    if (VERBOSE >= 2) {
        RCLCPP_INFO(this->get_logger(), "Time diff: '%f'", time_diff.count());    
        RCLCPP_INFO(this->get_logger(), "Rate: '%f'", rate);    
        RCLCPP_INFO(this->get_logger(), "Input: '%f'", input);    
        RCLCPP_INFO(this->get_logger(), "Output: '%f'", output);    
    }

    // Preparing time for next input
    previous_time = now;

    return output;

}
