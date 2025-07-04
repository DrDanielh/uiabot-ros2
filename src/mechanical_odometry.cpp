#include "mechanical_odometry.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include <chrono>

MechanicalOdometry::MechanicalOdometry() : Node("mechanical_odometry") {
  RCLCPP_INFO(this->get_logger(), "Instantiated mechanical_odometry node.");
  odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 10);
  axis0_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(axis0_vel_topic_,
                                                                            10,
                                                                            std::bind(&MechanicalOdometry::Axis0VelCallback_,
                                                                                      this,
                                                                                      std::placeholders::_1));
  axis1_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(axis1_vel_topic_,
                                                                            10,
                                                                            std::bind(&MechanicalOdometry::Axis1VelCallback_,
                                                                                      this,
                                                                                      std::placeholders::_1));
  update_timer_ = this->create_wall_timer(std::chrono::milliseconds(update_time_ms_),
                                          std::bind(&MechanicalOdometry::Update_, this));
  
  robot_tf_broadcaster_  = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


  this->declare_parameter<bool>("use_tf", false);
  use_tf_ = this->get_parameter("use_tf").as_bool();
}

void MechanicalOdometry::Update_() {
  // Calculate axis 0 and axis 1 angular position
  axis0_pos_ = axis0_pos_ + axis0_vel_*update_time_s_;
  axis1_pos_ = axis1_pos_ + axis1_vel_*update_time_s_;

  // Calculate left and right wheel velocity
  float left_wheel_vel = axis0_vel_/gear_ratio_;
  float right_wheel_vel = axis1_vel_/gear_ratio_;

  // Calculate forward kinematics
  robot_local_state_dot_ = ForwardKinematics_(left_wheel_vel, right_wheel_vel);

  // Integrate anglular velocity (robot yaw)
  robot_state_.theta = robot_state_.theta + robot_local_state_dot_.theta*update_time_s_;

  // Calculate global cartesian velocity
  robot_state_dot_.x = robot_local_state_dot_.x*cos(robot_state_.theta);
  robot_state_dot_.y = robot_local_state_dot_.x*sin(robot_state_.theta);

  // Integrate global cartesian velocity
  // TODO(martin): integration method may be improved
  robot_state_.x = robot_state_.x + robot_state_dot_.x*update_time_s_;
  robot_state_.y = robot_state_.y + robot_state_dot_.y*update_time_s_;
  
  // Update quaternion message after updated state
  // The quaternion message is used by both odometry publisher and tf broadcaster.
  UpdateQuaternionMsg();

  // Publish tf
  if (use_tf_) {
    PublishTf();
  }

  // Publish joint states
  PublishJointStates();

  // Publish odometry
  PublishOdometry();
}

RobotState MechanicalOdometry::ForwardKinematics_(const float& axis0_vel, const float& axis1_vel) {
  RobotState robot_state{};
  robot_state.x = wheel_radius_*((1.0/2.0)*axis0_vel + (1.0/2.0)*axis1_vel);
  robot_state.theta = wheel_radius_*(-(1.0/base_width_)*axis0_vel + (1.0/base_width_)*axis1_vel);
  return robot_state;
}

void MechanicalOdometry::UpdateQuaternionMsg() {
  tf2::Quaternion q;
  q.setRPY(0, 0, robot_state_.theta);
  quaternion_msg_.x = q.x();
  quaternion_msg_.y = q.y();
  quaternion_msg_.z = q.z();
  quaternion_msg_.w = q.w();
}

void MechanicalOdometry::PublishOdometry() {
  odom_msg_.header.stamp = this->get_clock()->now();
  odom_msg_.header.frame_id = odom_frame_;
  odom_msg_.child_frame_id = robot_frame_;
  odom_msg_.pose.pose.position.x = robot_state_.x;
  odom_msg_.pose.pose.position.y = robot_state_.y;
  odom_msg_.pose.pose.position.z = wheel_radius_;
  odom_msg_.pose.pose.orientation = quaternion_msg_;
  odom_msg_.twist.twist.linear.x = robot_local_state_dot_.x;
  odom_msg_.twist.twist.linear.y = 0.0;
  odom_msg_.twist.twist.angular.z = robot_local_state_dot_.theta;
  odometry_publisher_->publish(odom_msg_);
}

void MechanicalOdometry::PublishTf() {
  transform_stamped_msg_.header.stamp = this->get_clock()->now();
  transform_stamped_msg_.header.frame_id = odom_frame_;
  transform_stamped_msg_.child_frame_id = robot_frame_;
  transform_stamped_msg_.transform.translation.x = robot_state_.x;
  transform_stamped_msg_.transform.translation.y = robot_state_.y;
  transform_stamped_msg_.transform.translation.z = 0.0;
  transform_stamped_msg_.transform.rotation.x = quaternion_msg_.x;
  transform_stamped_msg_.transform.rotation.y = quaternion_msg_.y;
  transform_stamped_msg_.transform.rotation.z = quaternion_msg_.z;
  transform_stamped_msg_.transform.rotation.w = quaternion_msg_.w;
  robot_tf_broadcaster_->sendTransform(transform_stamped_msg_);
}

void MechanicalOdometry::PublishJointStates() {
  // Convert axis position to wheel position
  float left_wheel_pos = axis0_pos_/gear_ratio_;
  float right_wheel_pos = axis1_pos_/gear_ratio_;

  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = this->get_clock()->now();
  joint_state_msg.name = {"base_link_to_left_wheel", "base_link_to_right_wheel"};
  joint_state_msg.position = {left_wheel_pos, right_wheel_pos};
  joint_state_publisher_->publish(joint_state_msg);
}

void MechanicalOdometry::Axis0VelCallback_(const std_msgs::msg::Float32::SharedPtr msg) {
  axis0_vel_ = -msg->data;
}

void MechanicalOdometry::Axis1VelCallback_(const std_msgs::msg::Float32::SharedPtr msg) {
  axis1_vel_ = msg->data;
}

// Main function call
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MechanicalOdometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
