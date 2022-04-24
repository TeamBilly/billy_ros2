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

    // Parameters callback_mode_period. Define the /mode publish frequecy in ms.
    this->declare_parameter<int>("callback_mode_period", 10);
    this->get_parameter("callback_mode_period", p_callback_mode_period_);
    RCLCPP_INFO(this->get_logger(), "Init param 'callback_mode_period': '%d'",
      p_callback_mode_period_);

    // Parameters deadzone. Circular zone where the value of the joystick is set to 0.
    this->declare_parameter<float>("deadzone", 0.02);
    this->get_parameter("deadzone", p_deadzone_);
    RCLCPP_INFO(this->get_logger(), "Init param 'deadzone': '%f'", p_deadzone_);

    // Parameters straight_speed. Speed of the robot of in straight mode. 
    this->declare_parameter<float>("straight_speed", 0.5);
    this->get_parameter("straight_speed", p_straight_speed_);
    RCLCPP_INFO(this->get_logger(), "Init param 'straight_speed': '%f'", p_straight_speed_);

    // Parameters half_turn_speed. Speed of the robot of in half_speed mode. 
    this->declare_parameter<float>("half_turn_speed", 0.5);
    this->get_parameter("half_turn_speed", p_half_turn_speed_);
    RCLCPP_INFO(this->get_logger(), "Init param 'half_turn_speed': '%f'", p_half_turn_speed_);

}

void JoyManager::createPublishers()
{
    publisher_mode_ = this->create_publisher<std_msgs::msg::Int8>("/mode", 10);
    auto period_mode = p_callback_mode_period_ * 1ms;
    timer_mode_ = this->create_wall_timer(period_mode, std::bind(&JoyManager::callbackTimerMode, this));

    publisher_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto period_cmd_vel = p_callback_cmd_vel_period_ * 1ms;
    timer_cmd_vel_ = this->create_wall_timer(period_cmd_vel, std::bind(&JoyManager::callbackTimerCmdVel, this));
}

void JoyManager::createSubscribers()
{
    subscriber_joy_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoyManager::callbackJoy, this, _1));
}

void JoyManager::callbackTimerMode()
{
    auto message = std_msgs::msg::Int8();
    message.data = current_mode_;
    RCLCPP_INFO(this->get_logger(), "Publishing mode: '%d'", current_mode_);
    publisher_mode_->publish(message);
}

void JoyManager::callbackTimerCmdVel()
{
    // Adapt the motion depending on the mode
    applyMode();

    // Limit rate of values
    joy_turn_limited_ = rateLimiter(joy_turn_selected_, 
                                    joy_turn_limited_, 
                                    rate_limiter_previous_time_turn_);
    joy_speed_limited_ = rateLimiter(joy_speed_selected_, 
                                     joy_speed_limited_, 
                                     rate_limiter_previous_time_speed_);
    // Fill message
    auto message = geometry_msgs::msg::Twist();

    message.linear.x = joy_speed_limited_;
    message.angular.z = joy_turn_limited_;

    publisher_cmd_vel_->publish(message);
}

void JoyManager::applyMode()
{
    switch(current_mode_) {
        case BILLY_CONST.idle: {
                joy_turn_selected_ = 0.0;
                joy_speed_selected_ = 0.0;
                break;
            }
        case BILLY_CONST.teleop: {
                // Adjust to config values
                joy_speed_selected_ = joy_speed_ * BILLY_CONST.MOTION.LIN_X_MAX;
                joy_turn_selected_ = joy_turn_ * BILLY_CONST.MOTION.ANG_Z_MAX;
                break;
            }
        case BILLY_CONST.straight: {
                joy_speed_selected_ = p_straight_speed_;
                joy_turn_selected_ = 0.0;
                break;
            }
        case BILLY_CONST.half_turn: {
                joy_speed_selected_ = 0.0;
                joy_turn_selected_ = p_half_turn_speed_;
                break;
            }
    }
}

void JoyManager::callbackJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Fetching info
    // Left joystick info
    joy_speed_ = msg->axes[1];
    if((joy_speed_ < p_deadzone_) and (joy_speed_ > -p_deadzone_))
        joy_speed_ = 0;

    // Right joystick info
    joy_turn_ = msg->axes[3];
    if((joy_turn_ < p_deadzone_) and (joy_turn_ > -p_deadzone_))
        joy_turn_ = 0;

    // Up arrow is straight mode
    if((msg->axes[7] == 1.0)) {
        if (!rising_edge_up_arrow_)
            current_mode_ = BILLY_CONST.straight;
        rising_edge_up_arrow_ = true;
    } else {
        rising_edge_up_arrow_ = false;
    }

    // Down arrow is straight mode
    if((msg->axes[7] == -1.0)) {
        if (!rising_edge_down_arrow_)
            current_mode_ = BILLY_CONST.half_turn;
        rising_edge_down_arrow_ = true;
    } else {
        rising_edge_down_arrow_ = false;
    }

    // Left arrow is idle mode
    if((msg->axes[6] == 1.0)) {
        if (!rising_edge_left_arrow_)
            current_mode_ = BILLY_CONST.idle;
        rising_edge_left_arrow_ = true;
    } else {
        rising_edge_left_arrow_ = false;
    }

    // Right arrow is teleop mode
    if((msg->axes[6] == -1.0)) {
        if (!rising_edge_right_arrow_)
            current_mode_ = BILLY_CONST.teleop;
        rising_edge_right_arrow_ = true;
    } else {
        rising_edge_right_arrow_ = false;
    }


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
    if (p_verbose_ >= 2) {
        RCLCPP_INFO(this->get_logger(), "Time diff: '%f'", time_diff.count());    
        RCLCPP_INFO(this->get_logger(), "Rate: '%f'", rate);    
        RCLCPP_INFO(this->get_logger(), "Input: '%f'", input);    
        RCLCPP_INFO(this->get_logger(), "Output: '%f'", output);    
    }

    // Preparing time for next input
    previous_time = now;

    return output;

}
