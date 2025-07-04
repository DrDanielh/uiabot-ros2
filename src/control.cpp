#include <chrono>
#include <unistd.h>
#include "control.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

Control::Control() : Node("control") {
    RCLCPP_INFO(this->get_logger(), "Instantiated control node.");

    axis0_vel_ref_publisher_ = this->create_publisher<std_msgs::msg::Float32>(axis0_vel_ref_topic_, 10);
    axis1_vel_ref_publisher_ = this->create_publisher<std_msgs::msg::Float32>(axis1_vel_ref_topic_, 10);
    axis_state_client_ = this->create_client<odrive_interfaces::srv::AxisState>(axis_state_topic_);
    twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(twist_topic_, 
                                                                             10,
                                                                             std::bind(&Control::TwistCallback_,
                                                                                       this,
                                                                                       std::placeholders::_1));
    Active_();
}

bool Control::CallRequestState_(int axis, int state) {
  // Wait and check if service becomes available 
  RCLCPP_DEBUG(this->get_logger(), "Checking request_state service..");
  if (!axis_state_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "Unable to find request_state service. Make sure the node is running and has established connection with ODrive.");
    return false;
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Found request_state service.");
  }

  // Create and populate request message
  auto request = std::make_shared<odrive_interfaces::srv::AxisState::Request>();
  request->axis = axis;
  request->state = state;

  // Send request and wait for future
  RCLCPP_DEBUG(this->get_logger(), "Sending async request..");
  auto future = axis_state_client_->async_send_request(request);
  bool future_success = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS;

   // Check if futures returns usuccessful
  if (!future_success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to receive request_state service response.");
    return false;
  }
  
  // Get responses
  RCLCPP_DEBUG(this->get_logger(), "Checking response..");
  auto response_axis0 = future.get();

  // Check responses success
  RCLCPP_DEBUG(this->get_logger(), "Checking response success..");
  if (!response_axis0->success) {
    RCLCPP_ERROR(this->get_logger(), "Service request failed.");
    return false;
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Service request success.");
    return true;
  }
}

bool Control::RequestClosedLoop_() {
  bool success = true;
  success &= CallRequestState_(0, 8); // axis=0, state=8 (CLOSED_LOOP_CONTROL)
  success &= CallRequestState_(1, 8); // axis=1, state=8 (CLOSED_LOOP_CONTROL)
  return success;
}

void Control::Active_() {
  int conn_tries = 0;
  while (!RequestClosedLoop_() && (conn_tries++ < odrive_conn_timeout_)) {
    RCLCPP_INFO(this->get_logger(), "Could not establish connection with ODrive.");
    RCLCPP_INFO(this->get_logger(), "Retrying..");
    sleep(2);
  }

  if (conn_tries > odrive_conn_timeout_) {
    RCLCPP_ERROR(this->get_logger(), "ODrive connection timeout, exiting..");
    rclcpp::shutdown();
  }
}

void Control::TwistCallback_(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double robot_linear_vel = msg->linear.x;
    double robot_angular_vel = msg->angular.z;
    
    axis0_vel_ = (-(robot_linear_vel - (robot_angular_vel*base_width_)) / wheel_radius_) * gear_ratio;
    axis1_vel_ = ((robot_linear_vel + (robot_angular_vel*base_width_)) / wheel_radius_) * gear_ratio;

    auto axis0_vel_ref_msg = std_msgs::msg::Float32(); 
    auto axis1_vel_ref_msg = std_msgs::msg::Float32();

    axis0_vel_ref_msg.data = axis0_vel_;
    axis1_vel_ref_msg.data = axis1_vel_;

    axis0_vel_ref_publisher_->publish(axis0_vel_ref_msg);
    axis1_vel_ref_publisher_->publish(axis1_vel_ref_msg);
}

// Main function call
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Control>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}