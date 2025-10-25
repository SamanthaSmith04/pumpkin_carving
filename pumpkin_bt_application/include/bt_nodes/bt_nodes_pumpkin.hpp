#pragma once

#include <atomic>

#include <bt_common_ros_headers.hpp>
#include <pumpkin_msgs/srv/plan_motion.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp> 

#include <QFileDialog>
#include <yaml-cpp/yaml.h>
#include <fstream> 
#include "yaml_utils.hpp"


namespace bt_pumkin_nodes 
{
class LoadPosesListFromYAML : public BT::SyncActionNode
{
  public:
    LoadPosesListFromYAML(const std::string& name, const BT::NodeConfig& config) :
        BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick()
    {
      auto path_plan_file_path = getBTInput<std::string>(this, FILE_PATH_PORT_KEY);
      std::ifstream file(path_plan_file_path);
      if (!file.is_open()) {
          RCLCPP_ERROR(rclcpp::get_logger("LoadPosesListFromYAML"), "Failed to open pose array list file: %s", path_plan_file_path.c_str());
          return BT::NodeStatus::FAILURE;
      }
      YAML::Node yaml = YAML::Load(file);
      file.close();
      std::vector<geometry_msgs::msg::PoseArray> path_msg;
      try {
          if(!YAML::convert<std::vector<geometry_msgs::msg::PoseArray>>::decode(yaml, path_msg)) {
              RCLCPP_ERROR(rclcpp::get_logger("LoadPosesListFromYAML"), "Failed to decode path plan from YAML file: %s", path_plan_file_path.c_str());
              return BT::NodeStatus::FAILURE;
          }
      }
      catch (const YAML::Exception& e) {
          RCLCPP_ERROR(rclcpp::get_logger("LoadPosesListFromYAML"), "YAML parsing error: %s", e.what());
          return BT::NodeStatus::FAILURE;
      }
      setOutput(POSE_ARRAY_PORT_KEY, path_msg);
      RCLCPP_INFO(rclcpp::get_logger("LoadPosesListFromYAML"),
                  "Loaded path plan with %zu segments from file: %s",
                  path_msg.size(), path_plan_file_path.c_str());
      return BT::NodeStatus::SUCCESS;
    }

    inline static std::string FILE_PATH_PORT_KEY = "file_path";
    inline static std::string POSE_ARRAY_PORT_KEY = "pose_array_list";
    inline static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>(FILE_PATH_PORT_KEY),
                BT::OutputPort<std::vector<geometry_msgs::msg::PoseArray>>(POSE_ARRAY_PORT_KEY)};
    }
};

class GenerateMotionPlan : public BT::RosServiceNode<pumpkin_msgs::srv::PlanMotion>
{
public:

  inline static std::string POSE_ARRAY_LIST = "pose_list";
  inline static std::string TRAJECTORY_OUTPUT = "joint_trajectory";
  inline static std::string NUM_POINTS = "num_points";

  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<std::vector<geometry_msgs::msg::PoseArray>>(POSE_ARRAY_LIST), 
                              BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_OUTPUT),
                              BT::OutputPort<int>(NUM_POINTS)});
  }

  using RosServiceNode<pumpkin_msgs::srv::PlanMotion>::RosServiceNode;
  bool setRequest(typename Request::SharedPtr& request)
  {
    request->path = getBTInput<std::vector<geometry_msgs::msg::PoseArray>>(this, POSE_ARRAY_LIST);
    RCLCPP_INFO(logger(), "Sending pose array list with %zu segments", request->path.size());
    return true;
  }
  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response)
  {
      RCLCPP_INFO(logger(), "Response received for GenerateMotionPlan");
      if (response->success == false) {
          RCLCPP_ERROR(logger(), "Received empty trajectory in response");
          return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(logger(), "Received trajectory with %zu points", response->trajectory.points.size());
      setOutput(TRAJECTORY_OUTPUT, response->trajectory);
      setOutput(NUM_POINTS, static_cast<int>(response->trajectory.points.size()));
      return BT::NodeStatus::SUCCESS;
  }
};

class PubPoseArrayPreview : public BT::RosTopicPubNode<geometry_msgs::msg::PoseArray>
{
public:
  inline static std::string POSE_ARRAY_KEY = "pose_array";

  inline static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<std::vector<geometry_msgs::msg::PoseArray>>(POSE_ARRAY_KEY) });
  }

  PubPoseArrayPreview(const std::string& name, const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : RosTopicPubNode<geometry_msgs::msg::PoseArray>(name, conf, params) {}

  bool setMessage(geometry_msgs::msg::PoseArray& msg)
  {
    // get pose array from blackboard
    auto plan = getBTInput<std::vector<geometry_msgs::msg::PoseArray>>(this, POSE_ARRAY_KEY); 
    for (const auto& pose_array : plan) {
      for (const auto& pose : pose_array.poses) {
        msg.poses.push_back(pose);
      }
    }
    return true;
  }

};

}
