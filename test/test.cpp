/**
 * @file test.cpp
 * @author Sai Teja Gilukara (saitejag@umd.edu)
 * @brief Program for test cases
 * @version 0.1
 * @date 2023-11-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
/**
 * @brief Class for testing publisher
 * 
 */
class TaskPublisher : public testing::Test {
 protected:
  rclcpp::Node::SharedPtr test_node_;
};

/**
 * @brief Construct a new test f object
 * 
 */
TEST_F(TaskPublisher, test_num_publishers) {
  test_node_ = rclcpp::Node::make_shared("test_publisher");
  auto test_pub =
      test_node_->create_publisher<std_msgs::msg::String>("chatter", 10.0);

  auto number_of_publishers = test_node_->count_publishers("chatter");

  EXPECT_EQ(1, static_cast<int>(number_of_publishers));
}