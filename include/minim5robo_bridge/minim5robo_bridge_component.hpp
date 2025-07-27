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
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
// tf2
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
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
      USE_PC_TIME           = this->param<bool>("minim5robo_bridge.use_pc_time", true);
      image_pub_            = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", rclcpp::QoS(10));
      compressed_image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
          "/camera/compressed_image", rclcpp::QoS(10), [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            cv::Mat dst = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (dst.empty()) {
              RCLCPP_ERROR(this->get_logger(), "Failed to decode image");
              return;
            }
            // cv::imshow("b", dst);
            // cv::waitKey(0);
            sensor_msgs::msg::Image::SharedPtr image_msg_ptr = cv_bridge::CvImage(msg->header, "bgr8", dst).toImageMsg();
            image_pub_->publish(*image_msg_ptr);
          });
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10), [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        if (USE_PC_TIME)
          transform_stamped.header = ros2_utils::make_header(msg->header.frame_id, rclcpp::Clock().now());
        else
          transform_stamped.header = msg->header;
        RCLCPP_INFO(this->get_logger(), "base %s <-> odom %s", msg->child_frame_id.c_str(), msg->header.frame_id.c_str());
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
    // publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    // subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  };
} // namespace MiniM5Robo
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MiniM5Robo::Minim5RoboBridge)
