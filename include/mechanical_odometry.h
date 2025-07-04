/*!
 * Copyright (c) 2022. Dunder Mifflin, Inc.
 * All rights reserved.
 */

#ifndef UIABOT_INCLUDE_MECHANICAL_ODOMETRY_H_
#define UIABOT_INCLUDE_MECHANICAL_ODOMETRY_H_

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

struct RobotState {
  float x{};
  float y{};
  float theta{};
};

class MechanicalOdometry : public rclcpp::Node {
  public:
    MechanicalOdometry();

  private:
    /*! \brief Main update function.
    *
    *  Detailed description starts here.
    */
    void Update_();

    /*! \brief Calculate forward kinematics.
    *
    * TODO(martin): detailed description.
    * 
    * @param axis0_vel Angular velocity of axis0
    * @param axis1_vel Angular velocity of axis1
    * @returns Robot state in...
    */
    RobotState ForwardKinematics_(const float& axis0_vel, const float& axis1_vel);
    
    /*! \brief Update quaternion message.
    *
    * Update quaternion message based on the latest angular robot state.
    */
    void UpdateQuaternionMsg();

    /*! \brief Publish mechanical odometry.
    *
    * TODO(martin): detailed description.
    */
    void PublishOdometry();

    /*! \brief Publish static transform to tf.
     *
     * TODO(martin): detailed description.
     */
    void PublishStaticTf();
    
    /*! \brief Publish transform to tf.
    *
    * TODO(martin): detailed description.
    */
    void PublishTf();

    /*! \brief Publish joint states to robot state publisher.
    *
    * TODO(martin): detailed description.
    */
    void PublishJointStates();

    // Callback methods

    /*! \brief Axis 0 angular velocity callback.
    *
    * TODO(martin): detailed description.
    */
    void Axis0VelCallback_(const std_msgs::msg::Float32::SharedPtr msg);
    
    /*! \brief Axis 1 angular velocity callback.
    *
    * TODO(martin): detailed description.
    */
    void Axis1VelCallback_(const std_msgs::msg::Float32::SharedPtr msg);

    // Node parameters
    int update_time_ms_ = 10; // [ms]
    float update_time_s_ = update_time_ms_/1000.0; // update_time_ms_/1000.0; // [s]
    bool use_tf_{};

    // Vehicle parameters
    const float base_width_ = 0.39; // [m]
    const float wheel_radius_ = 0.055; // [m]
    const int gear_ratio_ = 20;

    // State variables
    float axis0_pos_{}; // left wheel angle
    float axis0_vel_{}; // left wheel velocity
    float axis1_pos_{}; // right wheel angle
    float axis1_vel_{}; // right wheel velocity

    RobotState robot_local_state_dot_{}; // robot_frame: x_dot, y_dot, theta_dot
    RobotState robot_state_{}; // world_frame: x, y, theta
    RobotState robot_state_dot_{}; // world frame: x_dot, y_dot, theta_dot

    // Frames
    std::string odom_frame_ = "odom";
    std::string robot_frame_ = "base_link";

    // Topics
    std::string odometry_topic_ = "mechanical_odometry";
    std::string joint_state_topic_ = "joint_states";
    std::string axis0_vel_topic_ = "axis0/vel";
    std::string axis1_vel_topic_ = "axis1/vel";

    // Publishers and subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr axis0_vel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr axis1_vel_subscriber_;

    // Tf broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> robot_tf_broadcaster_;

    // Messages
    geometry_msgs::msg::Quaternion quaternion_msg_;
    geometry_msgs::msg::TransformStamped transform_stamped_msg_;
    nav_msgs::msg::Odometry odom_msg_;

    // Timer
    rclcpp::TimerBase::SharedPtr update_timer_;
};

#endif // UIABOT_INCLUDE_MECHANICAL_ODOMETRY_H_