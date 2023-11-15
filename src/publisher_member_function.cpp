// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "beginner_tutorials/srv/msg_modify.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

using sharedFuture = rclcpp::Client<beginner_tutorials::srv::MsgModify>::SharedFuture;

/**
 * @brief MinimalPublisher class implementation.
 * provides a simple implementation of the MinimalPublisher which 
 * defines the talker, service client interface.
 * 
 */

class MinimalPublisher : public rclcpp::Node {
 public:
 /**
  * @brief Construct a new Minimal Publisher object
  * 
  */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
    param_desc.description = "Set callback frequency.";
    this->declare_parameter("freq", 2.0, param_desc);
    auto param = this->get_parameter("freq");
    auto freq = param.get_parameter_value().get<std::float_t>();
    RCLCPP_DEBUG(this->get_logger(),
                 "Parameter frequency is set to 2.0 hz");

    // creating a subscriber for tthe parameter
    parameter_subscriber_ =
        std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto parameterCallbackPtr =
        std::bind(&MinimalPublisher::parameter_callback, this, _1);
    parameterHandle_ = parameter_subscriber_->add_parameter_callback(
        "freq", parameterCallbackPtr);
    
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    
    // Logging information
    RCLCPP_DEBUG(this->get_logger(), "Publisher is Created");
    auto delta = std::chrono::milliseconds(static_cast<int>((1000 / freq)));
    timer_ = this->create_wall_timer(
        delta, std::bind(&MinimalPublisher::timer_callback, this));

    client = this->create_client<beginner_tutorials::srv::MsgModify>("modify_msg");

    // Logging information
    RCLCPP_DEBUG(this->get_logger(), "Client created");
    while (!client->wait_for_service(1s)) {
      // Log for interruption
      if (!rclcpp::ok()) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Interrupted");
        exit(EXIT_FAILURE);
      }
      // Logging information till waiting for the server
      RCLCPP_WARN(
          rclcpp::get_logger("rclcpp"),
          "NO SERVICE AVAILABLE, RUN SERVER TO REFLECT THE MODIFICATIONS");
    }
  }

 private:
  
  std::string Message;
  rclcpp::Client<beginner_tutorials::srv::MsgModify>::SharedPtr client;
  std::shared_ptr<rclcpp::ParameterEventHandler> parameter_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> parameterHandle_;

  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Teja! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  // Call service  at frequency of 10
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

  int call_service() {
    auto request = std::make_shared<beginner_tutorials::srv::MsgModify::Request>();
    request->a = "String1";
    request->b = " String2";
    // Logging information when service is called
    RCLCPP_INFO(this->get_logger(), "Calling Service to Modify string");
    auto callbackPtr =
        std::bind(&MinimalPublisher::response_callback, this, _1);
    client->async_send_request(request, callbackPtr);
    return 1;
  }

  void response_callback(sharedFuture future) {
    // Process the response
    RCLCPP_INFO(this->get_logger(), "Got String: %s", future.get()->c.c_str());
    Message = future.get()->c.c_str();
  }

  void parameter_callback(const rclcpp::Parameter &param) {
    RCLCPP_INFO(this->get_logger(),
                "cb: Received an update to parameter \"%s\" of type %s: %.2f",
                param.get_name().c_str(), param.get_type_name().c_str(),
                param.as_double());
    RCLCPP_WARN(this->get_logger(), "The base frequency has been changed");

    RCLCPP_FATAL_EXPRESSION(this->get_logger(), param.as_double() == 0.0, "Frequency is set to zero, zero division error");
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

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
