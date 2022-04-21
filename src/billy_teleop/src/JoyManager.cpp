#include "joy_manager.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

JoyManager::JoyManager() : Node("joy_manager_node"), count_(0)
{
    createPublishers();
    createSubscribers();
}

void JoyManager::createPublishers()
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&JoyManager::timer_callback, this));
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

void JoyManager::callbackJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    joy_speed_ = msg->axes[1];
    joy_turn_ = msg->axes[3];

    joy_speed_ *= SPEED_MAX;
    joy_turn_ *= TURN_MAX;
    joy_speed_previous_ = rateLimiter(joy_speed_, joy_speed_previous_);
}

float JoyManager::rateLimiter(float input, float output_previous)
{
    // Computing time difference
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_diff = 
      std::chrono::duration_cast<std::chrono::duration<double>>(now - rate_limiter_previous_time_);


    // Compute the rate
    float rate = (input - output_previous) / time_diff.count();

    // Evaluating rate
    float output = output_previous;
    if(rate > RISING_SLEW_RATE) {
        output = time_diff.count()*RISING_SLEW_RATE + output_previous;
    } else if (rate < FALLING_SLEW_RATE) {
        output = time_diff.count()*FALLING_SLEW_RATE + output_previous;
    } else {
        output = input;
    }

    if (VERBOSE >= 2) {
        RCLCPP_INFO(this->get_logger(), "Time diff: '%f'", time_diff.count());    
        RCLCPP_INFO(this->get_logger(), "Rate: '%f'", rate);    
        RCLCPP_INFO(this->get_logger(), "Input: '%f'", input);    
        RCLCPP_INFO(this->get_logger(), "Output: '%f'", output);    
    }

    // Preparing time for next input
    rate_limiter_previous_time_ = now;

    return output;

}
