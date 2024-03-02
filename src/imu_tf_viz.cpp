#include <chrono>
#include <unistd.h>
#include "imu_tf_viz.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


using namespace std::chrono_literals;

ImuTfViz::ImuTfViz() : Node("imu_tf_viz") {
    RCLCPP_INFO(this->get_logger(), "Instantiated imu_tf_viz node.");
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_data_topic_, 
                                                                       10,
                                                                       std::bind(&ImuTfViz::ImuCallback_,
                                                                                 this,
                                                                                 std::placeholders::_1));
    tf_broadcaster_  = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

                                                                                
}

void ImuTfViz::ImuCallback_(const sensor_msgs::msg::Imu::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped transform_stamped_msg;
    transform_stamped_msg.header.stamp = this->get_clock()->now();
    transform_stamped_msg.header.frame_id = parent_frame_;
    transform_stamped_msg.child_frame_id = child_frame_;
    transform_stamped_msg.transform.translation.x = 0.0;
    transform_stamped_msg.transform.translation.y = 0.0;
    transform_stamped_msg.transform.translation.z = 0.0;
    transform_stamped_msg.transform.rotation = msg->orientation;
    tf_broadcaster_->sendTransform(transform_stamped_msg);
}


// Main function call
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuTfViz>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}