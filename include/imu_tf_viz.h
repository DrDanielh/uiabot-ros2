/*!
 * Copyright (c) 2022. Dunder Mifflin, Inc.
 * All rights reserved.
 */

#ifndef UIABOT_INCLUDE_IMU_TF_VIZ_H_
#define UIABOT_INCLUDE_IMU_TF_VIZ_H_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"

class ImuTfViz : public rclcpp::Node {
  public:
    ImuTfViz();
  
  private:
    // Imu callback method
    void ImuCallback_(const sensor_msgs::msg::Imu::SharedPtr msg);

    // Imu data topic
    std::string imu_data_topic_ = "bno055/data";
    
    // Tf frames
    std::string parent_frame_ = "world";
    std::string child_frame_ = "imu";

    // Imu subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    
    // Tf broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif // UIABOT_INCLUDE_CONTROL_H_