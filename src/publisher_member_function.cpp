/**
 * @file publisher_member_function.cpp
 * @author Sai Teja Gilukara (saitejag@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2023-11-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/msg_modify.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


using sharedFuture = rclcpp::Client<
  beginner_tutorials::srv::MsgModify>::SharedFuture;

/**
 * @brief MinimalPublisher class implementation. It server client, talker node and logging functions.
 * 
 */
class MinimalPublisher : public rclcpp::Node {
 public:
 /**
  * @brief Construct a new Minimal Publisher object
  * 
  */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    // Declare parameters
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "Set callback frequency.";
    this->declare_parameter("freq", 2.0, param_desc);
    auto param = this->get_parameter("freq");
    auto freq = param.get_parameter_value().get<std::float_t>();
    RCLCPP_DEBUG(this->get_logger(),
                 "The parameter freq is declared, set to 2.0 hz");

    // Create subscriber for the parameters
    parameter_subscriber_ =
        std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto parameterCallbackPtr =
        std::bind(&MinimalPublisher::parameter_callback, this, _1);
    parameterHandle_ = parameter_subscriber_->add_parameter_callback(
        "freq", parameterCallbackPtr);

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // Logging information
    RCLCPP_DEBUG(this->get_logger(), "Created Publisher");
    auto delta = std::chrono::milliseconds(static_cast<int>((1000 / freq)));
    timer_ = this->create_wall_timer(
        delta, std::bind(&MinimalPublisher::timer_callback, this));

    client = this->create_client<
      beginner_tutorials::srv::MsgModify>("modify_msg");

    // Logging information
    RCLCPP_DEBUG(this->get_logger(), "Created Client");
    while (!client->wait_for_service(1s)) {
      // Log for interruption
      if (!rclcpp::ok()) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Interrupted");
        exit(EXIT_FAILURE);
      }
      // Logging information till waiting for the server
      RCLCPP_WARN(
          rclcpp::get_logger("rclcpp"),
          "RUN SERVER TO REFLECT CHANGES");
    }
    // Broadcast a tf frame
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";  // Parent "/world"
    t.child_frame_id = "talk";    // Child "/talk"

    // Translation block
    t.transform.translation.x = 1;
    t.transform.translation.y = 2;
    t.transform.translation.z = 3;

    // Rotation block
    t.transform.rotation.x = 1;
    t.transform.rotation.y = 0.5;
    t.transform.rotation.z = -1;
    t.transform.rotation.w = 0;

    tf_static_broadcaster_->sendTransform(t);
  }
  

 private:
  std::string Message;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::Client<beginner_tutorials::srv::MsgModify>::SharedPtr client;
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameterHandle_;

  /**
   * @brief publish message and calls server callback at every 10 count
   * 
   */
  void timer_callback() {
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Node setup");
    auto message = std_msgs::msg::String();
    message.data = "Teja! " + std::to_string(count_++);
    // Log message when service is available
    RCLCPP_INFO(this->get_logger(), "Publishing Node: '%s'",
                message.data.c_str());
    publisher_->publish(message);
    // Call service at the frequency of 10
    if (count_ % 10 == 0) {
      call_service();
    }
    auto steady_clock = rclcpp::Clock();
    RCLCPP_DEBUG_STREAM_THROTTLE(this->get_logger(), steady_clock, 10000,
                                 "Node running successfully");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  /**
   * @brief function to define service parameters
   * 
   * @return int 
   */
  int call_service() {
    auto request = std::make_shared<
      beginner_tutorials::srv::MsgModify::Request>();
    request->a = "String1";
    request->b = " String2";
    // Logging information when service is called
    RCLCPP_INFO(this->get_logger(), "Calling Service to Modify string");
    auto callbackPtr =
        std::bind(&MinimalPublisher::response_callback, this, _1);
    client->async_send_request(request, callbackPtr);
    return 1;
  }

  /**
   * @brief function call the response call_service method
   * 
   * @param future 
   */
  void response_callback(sharedFuture future) {
    // Process the response
    RCLCPP_INFO(this->get_logger(), "Got String: %s", future.get()->c.c_str());
    Message = future.get()->c.c_str();
  }

  /**
   * @brief function assigns updated value to parameter
   * 
   * @param param 
   */
  void parameter_callback(const rclcpp::Parameter &param) {
    RCLCPP_INFO(this->get_logger(),
                "cb: Received an update to parameter \"%s\" of type %s: %.2f",
                param.get_name().c_str(), param.get_type_name().c_str(),
                param.as_double());
    RCLCPP_WARN(this->get_logger(), "The base frequency has been changed");

    RCLCPP_FATAL_EXPRESSION(this->get_logger(), param.as_double() == 0.0,
                            "Frequency is set to zero, zero division error");
    if (param.as_double() == 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Frequency has not been changed.");
    } else {
      auto delta = std::chrono::milliseconds(
          static_cast<int>((1000 / param.as_double())));
      timer_ = this->create_wall_timer(
          delta, std::bind(&MinimalPublisher::timer_callback, this));
    }
  }
};

/**
 * @brief main function
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
