#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/mcpwm.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
#define max(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define min(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#define abs(a) ({ __typeof__ (a) _a = (a); _a > 0 ? _a : _a*-1; })
#define percent(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); ( _a / _b ) * 100; })
#define cross_product(a,b,c) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); __typeof__ (c) _c = (c);( _a / _b ) * _c; })

#define LED_BUILTIN 0
#define LED0 4

#define PIN_PWM_RIGHT 25
#define PIN_PWM_LEFT 26
#define PWM_MAX 100.0
#define PWM_MIN 0.0

#define DIR_PIN_RIGHT 32
#define DIR_PIN_LEFT 33

#define WHEEL_DIAMETER 0.065
#define WHEEL_DISTANCE 0.30
#define LIN_X_MAX 2.0
#define LIN_X_MIN 0.2

#define ANG_Z_MAX 3.14
#define ANG_Z_MIN 0.31

#define RAD_SEC_MAX 7.84
#define FREQ_MAX 100

rcl_publisher_t publisher;
rcl_publisher_t publisher_debug;
rcl_subscription_t subscriber;
rcl_subscription_t subscriber_state;
rcl_subscription_t subscriber_cmd_vel;
std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 msgin;
std_msgs__msg__Float32 msg_debug;
geometry_msgs__msg__Twist msgin_cmd_vel;

enum state{idle, teleop, half_turn, straight};
uint8_t state_current = idle;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        msg.data++;
    }
}

void timer_callback_debug(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        RCSOFTCHECK(rcl_publish(&publisher_debug, &msg_debug, NULL));
        // msg_debug.data++;
    }
}
// implementation example:
void subscription_callback(const void * msgin)
{
    // Cast received message to used type
    const std_msgs__msg__Int32 * freshmsg = (const std_msgs__msg__Int32 *)msgin;

    // Process message
    printf("Received: %d\n", freshmsg->data);

    RCSOFTCHECK(rcl_publish(&publisher, &freshmsg, NULL));
}

void callback_cmd_vel(const void * msgin_cmd_vel)
{
    // Cast received message to used type
    const geometry_msgs__msg__Twist * fmsg_cmd_vel = (const geometry_msgs__msg__Twist *)msgin_cmd_vel;

    // float rad_per_sec_r = ((2 * fmsg_cmd_vel->linear.x) + (WHEEL_DISTANCE * fmsg_cmd_vel->angular.z) ) / WHEEL_DIAMETER;
    // float rad_per_sec_l = ((2 * fmsg_cmd_vel->linear.x) - (WHEEL_DISTANCE * fmsg_cmd_vel->angular.z) ) / WHEEL_DIAMETER;

    // float adjusted_r = abs( (100.0f * rad_per_sec_r) / ANG_Z_MAX);
    // float adjusted_l = abs( (100.0f * rad_per_sec_l) / ANG_Z_MAX);

    // float pwm_r = max(min(adjusted_r, PWM_MAX), PWM_MIN);
    // float pwm_l = max(min(adjusted_l, PWM_MAX), PWM_MIN);

    float pwm_r = cross_product(fmsg_cmd_vel->linear.x, LIN_X_MAX, FREQ_MAX/2);
    float pwm_l = cross_product(fmsg_cmd_vel->linear.x, LIN_X_MAX, FREQ_MAX/2);
    pwm_r += cross_product(fmsg_cmd_vel->angular.z, ANG_Z_MAX, FREQ_MAX/2);
    pwm_l -= cross_product(fmsg_cmd_vel->angular.z, ANG_Z_MAX, FREQ_MAX/2);

    if(pwm_r >= 0.0) {
        gpio_set_level(DIR_PIN_RIGHT, 1);
    } else {
        gpio_set_level(DIR_PIN_RIGHT, 0);
    }
    
    if(pwm_l >= 0.0) {
        gpio_set_level(DIR_PIN_LEFT, 1);
    } else {
        gpio_set_level(DIR_PIN_LEFT, 0);
    }

    msg_debug.data = pwm_r;
    RCSOFTCHECK(rcl_publish(&publisher_debug, &msg_debug, NULL));

    if(abs(pwm_r) == 0.0) {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, 0.0f);
        mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, 1);
    } else {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, 60.0f);
        mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, (uint32_t)(abs(pwm_r)));
    }

    if(abs(pwm_l) == 0.0) {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1A, 0.0f);
        mcpwm_set_frequency(MCPWM_UNIT_1, MCPWM_TIMER_1, 1);
    } else {
        mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1A, 60.0f);
        mcpwm_set_frequency(MCPWM_UNIT_1, MCPWM_TIMER_1, (uint32_t)(abs(pwm_l)));
    }
}

void callback_state(const void * msgin_state)
{
    const std_msgs__msg__Int32 * cb_msgin_state = (const std_msgs__msg__Int32 *)msgin_state;
    state_current = cb_msgin_state->data;
    // Process message
    printf("Received: %d\n", state_current);

    /*
    switch(state_current) {
        case idle:
            gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, 99.0f);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0B, 99.0f);
            break;
        case teleop:
            gpio_set_level(LED0, !gpio_get_level(LED0));
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, 66.0f);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0B, 66.0f);
            break;
        case half_turn:
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, 33.0f);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0B, 33.0f);
            break;
        case straight:
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, 1.0f);
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0B, 1.0f);
            break;
    }
    */
}

void initPins(){
    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);

    gpio_reset_pin(LED0);
    gpio_set_direction(LED0, GPIO_MODE_INPUT_OUTPUT);

    gpio_reset_pin(DIR_PIN_RIGHT);
    gpio_set_direction(DIR_PIN_RIGHT, GPIO_MODE_OUTPUT);

    gpio_reset_pin(DIR_PIN_LEFT);
    gpio_set_direction(DIR_PIN_LEFT, GPIO_MODE_OUTPUT);

    gpio_reset_pin(PIN_PWM_RIGHT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_PWM_RIGHT);

    gpio_reset_pin(PIN_PWM_LEFT);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, PIN_PWM_LEFT);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, 60.0f);
    mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1A, 60.0f);
}

void appMain(void * arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "freertos_int32_publisher", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "freertos_int32_publisher"));

    RCCHECK(rclc_publisher_init_default(
      &publisher_debug,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "freertos_float32_publisher"));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "test_topic"));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
      &subscriber_cmd_vel,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel"));

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
      &subscriber_state,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "/billy/state"));

    // create timer,
    rcl_timer_t timer;
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

    rcl_timer_t timer_debug;
    const unsigned int timer_timeout_debug = 1000;
    RCCHECK(rclc_timer_init_default(
      &timer_debug,
      &support,
      RCL_MS_TO_NS(timer_timeout_debug),
      timer_callback_debug));

    // create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
    RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber, &msgin,
      &subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber_cmd_vel, &msgin_cmd_vel,
      &callback_cmd_vel, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber_state, &msgin,
      &callback_state, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_timer(&executor, &timer_debug));

    msg.data = 0;
    msgin.data = 0;
    msg.data = 0;
    msg_debug.data = 0;
    // msgin_cmd_vel.data = 0;

    initPins();

    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }

    // free resources
    RCCHECK(rcl_publisher_fini(&publisher, &node))
    RCCHECK(rcl_publisher_fini(&publisher_debug, &node))
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_subscription_fini(&subscriber_cmd_vel, &node));
    RCCHECK(rcl_node_fini(&node))

    vTaskDelete(NULL);
}
