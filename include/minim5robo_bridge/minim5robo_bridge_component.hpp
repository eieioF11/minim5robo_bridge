#pragma once
#include <execution>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
// ROS2
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
// tf2
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// extention node
#include "extension_node/extension_node.hpp"
// common_utils
#define USE_ROS2
#include "common_utils/common_utils.hpp"

#define _ENABLE_ATOMIC_ALIGNMENT_FIX

using namespace std::chrono_literals;

namespace MiniM5Robo {
  class Minim5RoboBridge : public ext_rclcpp::ExtensionNode {
  public:
    Minim5RoboBridge(const rclcpp::NodeOptions& options) : Minim5RoboBridge("", options) {}
    Minim5RoboBridge(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : ext_rclcpp::ExtensionNode("minim5robo_bridge_node", name_space, options), broadcaster_(this), tf_buffer_(this->get_clock()),
          listener_(tf_buffer_) {
      RCLCPP_INFO(this->get_logger(), "start minim5robo_bridge_node");
      USE_PC_TIME = this->param<bool>("minim5robo_bridge.use_pc_time", true);
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10), [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        if (USE_PC_TIME)
          transform_stamped.header = ros2_utils::make_header(msg->header.frame_id, rclcpp::Clock().now());
        else
          transform_stamped.header = msg->header;
        transform_stamped.child_frame_id          = msg->child_frame_id;
        transform_stamped.transform.translation.x = msg->pose.pose.position.x;
        transform_stamped.transform.translation.y = msg->pose.pose.position.y;
        transform_stamped.transform.translation.z = msg->pose.pose.position.z;
        transform_stamped.transform.rotation      = msg->pose.pose.orientation;
        broadcaster_.sendTransform(transform_stamped);
      });
    }

  private:
    bool USE_PC_TIME;
    // tf
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;
    // subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  };
} // namespace MiniM5Robo
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiniM5Robo::Minim5RoboBridge)
