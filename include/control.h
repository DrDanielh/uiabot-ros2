/*!
 * Copyright (c) 2022. Dunder Mifflin, Inc.
 * All rights reserved.
 */

#ifndef UIABOT_INCLUDE_CONTROL_H_
#define UIABOT_INCLUDE_CONTROL_H_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "odrive_interfaces/srv/axis_state.hpp"

class Control : public rclcpp::Node {
  public:
    Control();
  
  private:
    bool CallRequestState_(int axis, int state);
    bool RequestClosedLoop_();
    void Active_();

    // Callback methods
    void TwistCallback_(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Odrive
    bool odrive_connection_{};
    int odrive_axis0_state_{};
    int odrive_axis1_state_{};
    std::string odrive_axis0_msg_{};
    std::string odrive_axis1_msg_{};

    // Vehicle parameters
    double base_width_ = 0.185; // [m]
    double wheel_radius_ = 0.05; // [m]
    double gear_ratio = 20;

    // State variables
    double axis0_vel_{};
    double axis1_vel_{};

    // Node parameters
    int odrive_conn_timeout_ = 2; // num of tries

    // Topics
    std::string axis0_vel_ref_topic_ = "axis0/vel_ref";
    std::string axis1_vel_ref_topic_ = "axis1/vel_ref";
    std::string axis_state_topic_ = "request_state";
    std::string twist_topic_ = "cmd_vel";
    
    // Publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr axis0_vel_ref_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr axis1_vel_ref_publisher_;
    rclcpp::Client<odrive_interfaces::srv::AxisState>::SharedPtr axis_state_client_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
    
};

#endif // UIABOT_INCLUDE_CONTROL_H_