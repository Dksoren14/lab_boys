#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdomToTfNode : public rclcpp::Node
{
public:
  OdomToTfNode() : Node("odom_to_tf")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::QoS(10),
      std::bind(&OdomToTfNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "odom_to_tf node started");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = msg->header.frame_id;
    tf_msg.child_frame_id = msg->child_frame_id;

    tf_msg.transform.translation.x = msg->pose.pose.position.x;
    tf_msg.transform.translation.y = msg->pose.pose.position.y;
    tf_msg.transform.translation.z = msg->pose.pose.position.z;

    tf_msg.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTfNode>());
  rclcpp::shutdown();
  return 0;
}