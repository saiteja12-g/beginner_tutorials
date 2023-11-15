#include <beginner_tutorials/srv/msg_modify.hpp>
#include <cstdlib>
#include <iterator>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using MsgModify = beginner_tutorials::srv::MsgModify;

void add(const std::shared_ptr<MsgModify::Request> request,
         std::shared_ptr<MsgModify::Response> response) {
  response->c = "String has been modified to " + request->a + "-" + request->b;
}

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