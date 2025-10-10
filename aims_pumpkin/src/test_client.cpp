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

  for (int i = 0; i < 30; ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.345;
    pose.position.y = 0.0 + 0.02 * i;
    pose.position.z = 1.744 + 0.003 * i;
    pose.orientation.x = 0.7071068;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.7071068;
    pose.orientation.w = 0.0;

    pose_array.poses.push_back(pose);
  }

  // publish the PoseArray for visualization
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub
    = node->create_publisher<geometry_msgs::msg::PoseArray>("/test_poses", 10);
  pose_pub->publish(pose_array);
  RCLCPP_INFO(node->get_logger(), "Published test PoseArray with %zu poses", pose_array.poses.size());

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
    // Republish to trajectory preview

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub =
      node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/trajectory", 10);

    traj_pub->publish(response->trajectory);
    RCLCPP_INFO(node->get_logger(), "Published trajectory with %zu points", response->trajectory.points.size());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}