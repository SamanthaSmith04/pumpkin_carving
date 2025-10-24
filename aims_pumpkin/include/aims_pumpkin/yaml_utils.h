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

////// =============== STANDARD ROS TYPES =============== //////

// ---------- Pose ----------
template<>
struct convert<geometry_msgs::msg::Pose>
{
  static Node encode(const geometry_msgs::msg::Pose& rhs)
  {
    Node node;
    node["position"]["x"] = rhs.position.x;
    node["position"]["y"] = rhs.position.y;
    node["position"]["z"] = rhs.position.z;
    node["orientation"]["x"] = rhs.orientation.x;
    node["orientation"]["y"] = rhs.orientation.y;
    node["orientation"]["z"] = rhs.orientation.z;
    node["orientation"]["w"] = rhs.orientation.w;
    return node;
  }

  static bool decode(const Node& node, geometry_msgs::msg::Pose& rhs)
  {
    if (!node["position"] || !node["orientation"]) {
      return false;
    }
    rhs.position.x = node["position"]["x"].as<double>();
    rhs.position.y = node["position"]["y"].as<double>();
    rhs.position.z = node["position"]["z"].as<double>();
    rhs.orientation.x = node["orientation"]["x"].as<double>();
    rhs.orientation.y = node["orientation"]["y"].as<double>();
    rhs.orientation.z = node["orientation"]["z"].as<double>();
    rhs.orientation.w = node["orientation"]["w"].as<double>();
    return true;
  }
};

// ---------- Header ----------
template<>
struct convert<std_msgs::msg::Header>
{
  static Node encode(const std_msgs::msg::Header& rhs)
  {
    Node node;
    node["stamp"]["sec"] = rhs.stamp.sec;
    node["stamp"]["nanosec"] = rhs.stamp.nanosec;
    node["frame_id"] = rhs.frame_id;
    return node;
  }

  static bool decode(const Node& node, std_msgs::msg::Header& rhs)
  {
    if (!node["stamp"] || !node["frame_id"]) {
      return false;
    }
    rhs.stamp.sec = node["stamp"]["sec"].as<uint32_t>();
    rhs.stamp.nanosec = node["stamp"]["nanosec"].as<uint32_t>();
    rhs.frame_id = node["frame_id"].as<std::string>();
    return true;
  }
};

// ---------- PoseArray ----------
template<>
struct convert<geometry_msgs::msg::PoseArray>
{
  static Node encode(const geometry_msgs::msg::PoseArray& rhs)
  {
    Node node;
    node["header"] = YAML::convert<std_msgs::msg::Header>::encode(rhs.header);
    for (const auto& pose : rhs.poses) {
      node["poses"].push_back(YAML::convert<geometry_msgs::msg::Pose>::encode(pose));
    }
    return node;
  }

  static bool decode(const Node& node, geometry_msgs::msg::PoseArray& rhs)
  {
    if (!node["header"] || !node["poses"] || !node["poses"].IsSequence()) {
      return false;
    }
    rhs.header = node["header"].as<std_msgs::msg::Header>();
    rhs.poses.clear();
    for (const auto& n : node["poses"]) {
      rhs.poses.push_back(n.as<geometry_msgs::msg::Pose>());
    }
    return true;
  }
};

// ---------- JointTrajectoryPoint ----------
template<>
struct convert<trajectory_msgs::msg::JointTrajectoryPoint>
{
  static Node encode(const trajectory_msgs::msg::JointTrajectoryPoint& rhs)
  {
    Node node;
    for (const auto& position : rhs.positions) {
      node["positions"].push_back(position);
    }
    for (const auto& velocity : rhs.velocities) {
      node["velocities"].push_back(velocity);
    }
    if (rhs.velocities.empty()) {
      for (const auto& acceleration : rhs.positions) {
        node["velocities"].push_back(0.0);
      }
    }
    for (const auto& acceleration : rhs.accelerations) {
      node["accelerations"].push_back(acceleration);
    }
    if (rhs.accelerations.empty()) {
      for (const auto& acceleration : rhs.positions) {
        node["accelerations"].push_back(0.0);
      }
    }
    for (const auto& effort : rhs.effort) {
      node["effort"].push_back(effort);
    }
      if (rhs.effort.empty()) {
        for (const auto& effort : rhs.positions) {
          node["effort"].push_back(0.0);
        }
      }
    node["time_from_start"]["sec"] = rhs.time_from_start.sec;
    node["time_from_start"]["nanosec"] = rhs.time_from_start.nanosec;
    return node;
  }

  static bool decode(const Node& node, trajectory_msgs::msg::JointTrajectoryPoint& rhs)
  {
    if (!node["positions"] || !node["velocities"] || !node["accelerations"] ||
        !node["effort"] || !node["time_from_start"]) {
      return false;
    }
    rhs.positions.clear();
    for (const auto& n : node["positions"]) {
      rhs.positions.push_back(n.as<double>());
    }
    rhs.velocities.clear();
    for (const auto& n : node["velocities"]) {
      rhs.velocities.push_back(n.as<double>());
    }
    rhs.accelerations.clear();
    for (const auto& n : node["accelerations"]) {
      rhs.accelerations.push_back(n.as<double>());
    }
    rhs.effort.clear();
    for (const auto& n : node["effort"]) {
      rhs.effort.push_back(n.as<double>());
    }
    rhs.time_from_start.sec = node["time_from_start"]["sec"].as<uint32_t>();
    rhs.time_from_start.nanosec = node["time_from_start"]["nanosec"].as<uint32_t>();
    return true;
  }
};

// ---------- Joint Trajectory ----------
template<>
struct convert<trajectory_msgs::msg::JointTrajectory>
{
  static Node encode(const trajectory_msgs::msg::JointTrajectory& rhs)
  {
    Node node;
    node["header"] = YAML::convert<std_msgs::msg::Header>::encode(rhs.header);
    for (const auto& joint_name : rhs.joint_names) {
      node["joint_names"].push_back(joint_name);
    }
    for (const auto& point : rhs.points) {
      node["points"].push_back(YAML::convert<trajectory_msgs::msg::JointTrajectoryPoint>::encode(point));
    }
    return node;
  }

  static bool decode(const Node& node, trajectory_msgs::msg::JointTrajectory& rhs)
  {
    if (!node["header"] || !node["joint_names"] || !node["joint_names"].IsSequence() ||
        !node["points"] || !node["points"].IsSequence()) {
      return false;
    }
    rhs.header = node["header"].as<std_msgs::msg::Header>();
    rhs.joint_names.clear();
    for (const auto& n : node["joint_names"]) {
      rhs.joint_names.push_back(n.as<std::string>());
    }
    rhs.points.clear();
    for (const auto& n : node["points"]) {
      rhs.points.push_back(n.as<trajectory_msgs::msg::JointTrajectoryPoint>());
    }
    return true;
  }
};

// ================== CUSTOM TYPES =====================
template<>
struct convert<std::vector<geometry_msgs::msg::PoseArray>> 
{
  static Node encode(const std::vector<geometry_msgs::msg::PoseArray>& rhs)
  {
    Node node;
    for (const auto& pose_array : rhs) {
      node.push_back(YAML::convert<geometry_msgs::msg::PoseArray>::encode(pose_array));
    }
    return node;
  }
  static bool decode(const Node& node, std::vector<geometry_msgs::msg::PoseArray>& rhs)
  {
    if (!node.IsSequence()) {
      return false;
    }
    rhs.clear();
    for (const auto& n : node) {
      rhs.push_back(n.as<geometry_msgs::msg::PoseArray>());
    }
    return true;
  }
};

} // namespace YAML
