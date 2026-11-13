#pragma once

#include <atomic>

#include <bt_common_ros_headers.hpp>
#include <pumpkin_msgs/srv/plan_motion.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp> 

#include <ament_index_cpp/get_package_share_directory.hpp>


#include <QComboBox>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


#include <QFileDialog>
#include <yaml-cpp/yaml.h>
#include <fstream> 
#include "yaml_utils.hpp"


namespace bt_pumkin_nodes 
{

class GetTFPose : public BT::SyncActionNode
{
public:
    GetTFPose(const std::string& name, const BT::NodeConfig& config,
              std::shared_ptr<tf2_ros::Buffer> tf_buffer)
        : BT::SyncActionNode(name, config), tf_buffer_(tf_buffer) {}

    BT::NodeStatus tick() override
    {
        std::string target_frame, source_frame;
        if (!getInput(TARGET_FRAME, target_frame) || !getInput(SOURCE_FRAME, source_frame)) {
            RCLCPP_ERROR(rclcpp::get_logger("GetTFPose"), "Missing target_frame or source_frame input!");
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::Pose pose_output;
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
            
            pose_output.position.x = transform_stamped.transform.translation.x;
            pose_output.position.y = transform_stamped.transform.translation.y;
            pose_output.position.z = transform_stamped.transform.translation.z;
            pose_output.orientation.x = transform_stamped.transform.rotation.x;
            pose_output.orientation.y = transform_stamped.transform.rotation.y;
            pose_output.orientation.z = transform_stamped.transform.rotation.z;
            pose_output.orientation.w = transform_stamped.transform.rotation.w;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(rclcpp::get_logger("GetTFPose"), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
            return BT::NodeStatus::FAILURE;
        }

        // Write the result to the output port
        setOutput(POSE_OUTPUT, pose_output);

        RCLCPP_INFO(rclcpp::get_logger("GetTFPose"), "Successfully transformed %s to %s", source_frame.c_str(), target_frame.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    inline static const std::string TARGET_FRAME = "target_frame";
    inline static const std::string SOURCE_FRAME = "source_frame";
    inline static const std::string POSE_OUTPUT = "pose_output";

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>(TARGET_FRAME), 
            BT::InputPort<std::string>(SOURCE_FRAME),
            BT::OutputPort<geometry_msgs::msg::Pose>(POSE_OUTPUT)
        };
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

class GetComboBoxValue : public BT::SyncActionNode
{
    public:
        GetComboBoxValue(const std::string& name, const BT::NodeConfig& config) :
            BT::SyncActionNode(name, config) {}
        BT::NodeStatus tick()
        {
            auto combo_box_name = getBTInput<std::string>(this, COMBO_BOX_PORT_KEY);
            auto* combo_box = this->config().blackboard->get<QComboBox*>(combo_box_name);
            if (!combo_box) {
                RCLCPP_ERROR(rclcpp::get_logger("GetComboBoxValue"), "ComboBox pointer is null");
                return BT::NodeStatus::FAILURE;
            }
            int current_index = combo_box->currentIndex();
            setOutput(COMBO_BOX_VALUE_PORT_KEY, current_index);
            return BT::NodeStatus::SUCCESS;
        }

        inline static std::string COMBO_BOX_PORT_KEY = "combo_box";
        inline static std::string COMBO_BOX_VALUE_PORT_KEY = "combo_box_value";

        inline static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::string>(COMBO_BOX_PORT_KEY), 
                                       BT::OutputPort<int>(COMBO_BOX_VALUE_PORT_KEY)};
        }
};

class GenerateRoboDKMotionPlan : public BT::RosServiceNode<pumpkin_msgs::srv::PlanMotion>
{
    public:
        GenerateRoboDKMotionPlan(const std::string& name, 
                                 const BT::NodeConfig& config, 
                                 const BT::RosNodeParams& params)
            : BT::RosServiceNode<pumpkin_msgs::srv::PlanMotion>(name, config, params) {}

        bool setRequest(typename Request::SharedPtr& request)
        {
            request->path = getBTInput<std::vector<geometry_msgs::msg::PoseArray>>(this, POSE_ARRAY_LIST);
            RCLCPP_INFO(logger(), "Sending pose array list with %zu segments", request->path.size());
            return true;
        }

        BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response)
        {
            RCLCPP_INFO(logger(), "Response received for GenerateRoboDKMotionPlan");
            if (response->success == false) {
                RCLCPP_ERROR(logger(), "Received empty trajectory in response");
                return BT::NodeStatus::FAILURE;
            }
            RCLCPP_INFO(logger(), "Received trajectory with %zu points", response->trajectory.points.size());
            setOutput(TRAJECTORY_OUTPUT, response->trajectory);
            setOutput(NUM_POINTS, static_cast<int>(response->trajectory.points.size()));
            return BT::NodeStatus::SUCCESS;
        }
    
        inline static std::string POSE_ARRAY_LIST = "pose_list";
        inline static std::string PUMPKIN_FACE_FRAME = "pumpkin_face_frame";
        inline static std::string TRAJECTORY_OUTPUT = "joint_trajectory";
        inline static std::string NUM_POINTS = "num_points";

        inline static BT::PortsList providedPorts()
        {
            return providedBasicPorts({
                BT::InputPort<std::vector<geometry_msgs::msg::PoseArray>>(POSE_ARRAY_LIST), 
                BT::InputPort<geometry_msgs::msg::Pose>(PUMPKIN_FACE_FRAME),
                BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_OUTPUT),
                BT::OutputPort<int>(NUM_POINTS)
            });
        }
};
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
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = "pumpkin_face";

    return true;
  }

};

}
