#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_pose_array_client");
  auto publisher = node->create_publisher<geometry_msgs::msg::PoseArray>(
    "/motion_planning_client/pose_array", 10);

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = node->now();
  pose_array.header.frame_id = "map";

  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  pose_array.poses.push_back(pose);

  rclcpp::Rate rate(1);
  int count = 0;
  while (rclcpp::ok() && count < 5) {
    pose_array.header.stamp = node->now();
    publisher->publish(pose_array);
    RCLCPP_INFO(node->get_logger(), "Published PoseArray");
    rate.sleep();
    count++;
  }

  rclcpp::shutdown();
  return 0;
}