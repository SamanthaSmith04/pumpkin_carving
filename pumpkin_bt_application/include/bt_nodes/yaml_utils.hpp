#pragma once

#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/header.hpp>
#ifdef HAS_CAPSEN_SUPPORT
#include <plg_msgs/msg/path_segment.hpp>
#include <plg_msgs/msg/tool_path.hpp>
#endif
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace YAML
{
template<>
struct convert<std::vector<geometry_msgs::msg::PoseArray>> 
{
  static Node encode(const std::vector<geometry_msgs::msg::PoseArray>& rhs)
  {
    Node node;
    Node path_node;
    for (const auto& pose_array : rhs)
      path_node.push_back(YAML::convert<geometry_msgs::msg::PoseArray>::encode(pose_array));
    node["path"] = path_node;
    return node;
  }
  static bool decode(const Node& node, std::vector<geometry_msgs::msg::PoseArray>& rhs)
  {
    if (!node["path"] || !node["path"].IsSequence()) {
      return false;
    }
    rhs.clear();
    for (const auto& n : node["path"]) {
      rhs.push_back(n.as<geometry_msgs::msg::PoseArray>());
    }
    return true;
  }
};
}