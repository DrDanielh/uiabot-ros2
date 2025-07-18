#include "wheel_tf_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

WheelTfPublisher::WheelTfPublisher() : Node("wheel_tf_publisher") {
  RCLCPP_INFO(this->get_logger(), "Instantiated wheel_tf_publisher node.");
  
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);
  
  axis0_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(axis0_vel_topic_,
                                                                            10,
                                                                            std::bind(&WheelTfPublisher::Axis0VelCallback_,
                                                                                      this,
                                                                                      std::placeholders::_1));
  axis1_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(axis1_vel_topic_,
                                                                            10,
                                                                            std::bind(&WheelTfPublisher::Axis1VelCallback_,
                                                                                      this,
                                                                                      std::placeholders::_1));
  
  update_timer_ = this->create_wall_timer(std::chrono::milliseconds(update_time_ms_),
                                          std::bind(&WheelTfPublisher::Update_, this));
}

void WheelTfPublisher::Update_() {
  // Calculate axis 0 and axis 1 angular position
  axis0_pos_ = axis0_pos_ + axis0_vel_*update_time_s_;
  axis1_pos_ = axis1_pos_ + axis1_vel_*update_time_s_;

  // Publish joint states
  PublishJointStates();
}

void WheelTfPublisher::PublishJointStates() {
  // Convert axis position to wheel position
  float left_wheel_pos = axis0_pos_/gear_ratio_;
  float right_wheel_pos = axis1_pos_/gear_ratio_;

  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = this->get_clock()->now();
  joint_state_msg.name = {"base_link_to_left_wheel", "base_link_to_right_wheel"};
  joint_state_msg.position = {left_wheel_pos, right_wheel_pos};
  joint_state_publisher_->publish(joint_state_msg);
}

void WheelTfPublisher::Axis0VelCallback_(const std_msgs::msg::Float32::SharedPtr msg) {
  axis0_vel_ = -msg->data;
}

void WheelTfPublisher::Axis1VelCallback_(const std_msgs::msg::Float32::SharedPtr msg) {
  axis1_vel_ = msg->data;
}

// Main function call
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelTfPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}