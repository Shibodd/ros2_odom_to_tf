#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <builtin_interfaces/msg/time.hpp>

#include "parameters.hpp"

class TheNode : public rclcpp::Node {
  bool m_override_stamp;
  std::optional<std::string> m_override_frame_id;

  std::string m_child_frame_id;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;

  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

public:
  TheNode() : rclcpp::Node("ros2_odom_to_tf") {
    Parameters p(this);

    m_child_frame_id = p.get<std::string>("child_frame_id");

    m_override_frame_id = p.get_maybe<std::string>("override_frame_id_with");
    m_override_stamp = p.get<bool>("override_stamp_with_clock", false);

    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      p.get<std::string>("odometry_topic"),
      p.parse_qos("odometry_qos_override"),
      std::bind(&TheNode::odom_cb, this, std::placeholders::_1)
    );

    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  void odom_cb(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    geometry_msgs::msg::TransformStamped tf;
    
    tf.header.frame_id = m_override_frame_id.value_or(msg->header.frame_id);
    tf.header.stamp = m_override_stamp? 
      builtin_interfaces::msg::Time(this->get_clock()->now()) 
      : msg->header.stamp;

    tf.child_frame_id = m_child_frame_id;
    tf.transform.translation
      .set__x(msg->pose.pose.position.x)
      .set__y(msg->pose.pose.position.y)
      .set__z(msg->pose.pose.position.z);

    tf.transform.rotation
      .set__w(msg->pose.pose.orientation.w)
      .set__x(msg->pose.pose.orientation.x)
      .set__y(msg->pose.pose.orientation.y)
      .set__z(msg->pose.pose.orientation.z);

    m_tf_broadcaster->sendTransform(tf);
  }
};

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TheNode>());
  rclcpp::shutdown();
  return 0;
}