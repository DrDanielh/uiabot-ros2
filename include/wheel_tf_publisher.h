/*!
 * Copyright (c) 2022. Dunder Mifflin, Inc.
 * All rights reserved.
 */

#ifndef UIABOT_INCLUDE_WHEEL_TF_PUBLISHER_H_
#define UIABOT_INCLUDE_WHEEL_TF_PUBLISHER_H_

#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"

class WheelTfPublisher : public rclcpp::Node {
  public:
    WheelTfPublisher();

  private:
    void Update_();
    void PublishJointStates();
    void Axis0VelCallback_(const std_msgs::msg::Float32::SharedPtr msg);
    void Axis1VelCallback_(const std_msgs::msg::Float32::SharedPtr msg);

    // Node parameters
    int update_time_ms_ = 10; // [ms]
    float update_time_s_ = update_time_ms_/1000.0; // [s]

    // Vehicle parameters
    const int gear_ratio_ = 20;

    // State variables
    float axis0_pos_{}; // left wheel angle
    float axis0_vel_{}; // left wheel velocity
    float axis1_pos_{}; // right wheel angle
    float axis1_vel_{}; // right wheel velocity

    // Topics
    std::string joint_state_topic_ = "joint_states";
    std::string axis0_vel_topic_ = "axis0/vel";
    std::string axis1_vel_topic_ = "axis1/vel";

    // Publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr axis0_vel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr axis1_vel_subscriber_;

    // Timer
    rclcpp::TimerBase::SharedPtr update_timer_;
};

#endif // UIABOT_INCLUDE_WHEEL_TF_PUBLISHER_H_