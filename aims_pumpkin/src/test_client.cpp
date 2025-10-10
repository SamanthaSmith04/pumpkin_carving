#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <pumpkin_msgs/srv/plan_motion.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("test_motion_plan_client");

  auto client = node->create_client<pumpkin_msgs::srv::PlanMotion>("/generate_motion_plan");

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = node->now();
  pose_array.header.frame_id = "world";

  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.345;
  pose.position.y = 0.0;
  pose.position.z = 1.744;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  pose_array.poses.push_back(pose);

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 1.345;
  pose2.position.y = 0.02;
  pose2.position.z = 1.747;
  pose2.orientation.x = 0.0;
  pose2.orientation.y = 0.0;
  pose2.orientation.z = 0.0;
  pose2.orientation.w = 1.0;

  pose_array.poses.push_back(pose2);

  // Wait for service to be available
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Service not available");
    rclcpp::shutdown();
    return 1;
  }

  auto request = std::make_shared<pumpkin_msgs::srv::PlanMotion::Request>();
  request->path.push_back(pose_array);

  auto future = client->async_send_request(request);

  // Wait for the result
  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    RCLCPP_INFO(node->get_logger(), "Received motion plan response");
    // Process response as needed
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}