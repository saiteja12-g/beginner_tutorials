/**
 * @file server.cpp
 * @author Sai Teja Gilukara (saitejag@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2023-11-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <beginner_tutorials/srv/msg_modify.hpp>
#include <cstdlib>
#include <iterator>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using MsgModify = beginner_tutorials::srv::MsgModify;

/**
 * @brief Function for modifying the message by concatenating
 * 
 * @param request 
 * @param response 
 */
void add(const std::shared_ptr<MsgModify::Request> request,
         std::shared_ptr<MsgModify::Response> response) {
  response->c = "String has been modified to " + request->a + "-" + request->b;
}

/**
 * @brief main function
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("modify_msg_server");

  rclcpp::Service<MsgModify>::SharedPtr service =
      node->create_service<MsgModify>("modify_msg", &add);

  // Logging information to show when the server is started
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Server Started.. Modifying Msg");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
