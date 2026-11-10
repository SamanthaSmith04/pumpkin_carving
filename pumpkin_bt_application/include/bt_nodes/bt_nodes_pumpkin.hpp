#pragma once

#include <atomic>

#include <bt_common_ros_headers.hpp>
#include <pumpkin_msgs/srv/plan_motion.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp> 

#define PY_SSIZE_T_CLEAN
#undef slots
#include <Python.h>
#define slots Q_SLOTS
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
    GetTFPose(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    BT::NodeStatus tick()
    {
        auto node = rclcpp::Node::make_shared("get_tf_pose_node");
        auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        std::string target_frame = getBTInput<std::string>(this, TARGET_FRAME);
        std::string source_frame = getBTInput<std::string>(this, SOURCE_FRAME);

        geometry_msgs::msg::Pose pose_output;
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            pose_output.position.x = transform_stamped.transform.translation.x;
            pose_output.position.y = transform_stamped.transform.translation.y;
            pose_output.position.z = transform_stamped.transform.translation.z;
            pose_output.orientation.x = transform_stamped.transform.rotation.x;
            pose_output.orientation.y = transform_stamped.transform.rotation.y;
            pose_output.orientation.z = transform_stamped.transform.rotation.z;
            pose_output.orientation.w = transform_stamped.transform.rotation.w;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(node->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
            return BT::NodeStatus::FAILURE;
        }

    }

    inline static std::string TARGET_FRAME = "target_frame";
    inline static std::string SOURCE_FRAME = "source_frame";
    inline static std::string POSE_OUTPUT = "pose_output";

    inline static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>(TARGET_FRAME), 
                                       BT::InputPort<std::string>(SOURCE_FRAME),
                                       BT::OutputPort<geometry_msgs::msg::Pose>(POSE_OUTPUT)};
    }
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

class GenerateRoboDKMotionPlan : public BT::SyncActionNode
{
    public:
        GenerateRoboDKMotionPlan(const std::string& name, const BT::NodeConfig& config) :
            BT::SyncActionNode(name, config) {}
        BT::NodeStatus tick()
        {
            Py_Initialize();

            std::string package_share_directory = ament_index_cpp::get_package_share_directory("aims_pumpkin");
            std::string script_path = package_share_directory + "/scripts/motion_planning.py";
            std::string py_code = "import sys\nsys.path.append('" + script_path + "')";
            PyRun_SimpleString(py_code.c_str());
            
            PyObject* pName = PyUnicode_DecodeFSDefault("motion_planning");
            PyObject* pModule = PyImport_Import(pName);
            Py_DECREF(pName);

            if (pModule != nullptr) {
                PyObject* pFunc = PyObject_GetAttrString(pModule, "_plan_motion");

                if (pFunc && PyCallable_Check(pFunc)) {
                    // Pack arguments into tuple
                    PyObject* pArgs = PyTuple_New(2);
                    PyObject* pPoseList = PyList_New(0);
                    auto pose_list = getBTInput<std::vector<geometry_msgs::msg::PoseArray>>(this, POSE_ARRAY_LIST);
                    for (const auto& pose_array : pose_list) {
                        PyObject* pPoseArray = PyList_New(0);
                        for (const auto& pose : pose_array.poses) {
                            PyObject* pPose = Py_BuildValue("{s:{s:f,s:f,s:f},s:{s:f,s:f,s:f,s:f}}",
                                "position", "x", pose.position.x, "y", pose.position.y, "z", pose.position.z,
                                "orientation", "x", pose.orientation.x, "y", pose.orientation.y, "z", pose.orientation.z, "w", pose.orientation.w);
                            PyList_Append(pPoseArray, pPose);
                            Py_DECREF(pPose);
                        }
                        PyList_Append(pPoseList, pPoseArray);
                        Py_DECREF(pPoseArray);
                    }
                    auto pumpkin_face_frame = getBTInput<geometry_msgs::msg::Pose>(this, PUMPKIN_FACE_FRAME);
                    auto pPumpkinFaceFrame = Py_BuildValue("{s:{s:f,s:f,s:f},s:{s:f,s:f,s:f,s:f}}",
                        "position", "x", pumpkin_face_frame.position.x, "y", pumpkin_face_frame.position.y, "z", pumpkin_face_frame.position.z,
                        "orientation", "x", pumpkin_face_frame.orientation.x, "y", pumpkin_face_frame.orientation.y, "z", pumpkin_face_frame.orientation.z, "w", pumpkin_face_frame.orientation.w);
                    Py_DECREF(pPumpkinFaceFrame);

                    PyTuple_SetItem(pArgs, 0, pPoseList);
                    PyTuple_SetItem(pArgs, 1, pPumpkinFaceFrame);

                    PyObject* pValue = PyObject_CallObject(pFunc, pArgs);
                    Py_DECREF(pArgs);
                    Py_DECREF(pValue);

                    if (pValue != nullptr) {
                        // Convert the returned Python object to a ROS message
                        trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
                        // Conversion logic here (depends on how the Python function returns the data)
                        // For example, if pValue is a list of joint positions, you would iterate over it and fill the joint_trajectory_msg

                        setOutput(TRAJECTORY_OUTPUT, joint_trajectory_msg);
                        return BT::NodeStatus::SUCCESS;
                    } else {
                        PyErr_Print();
                        RCLCPP_ERROR(rclcpp::get_logger("GenerateRoboDKMotionPlan"), "Python function call failed");
                        return BT::NodeStatus::FAILURE;
                    }
                }
            }
        }

        inline static std::string POSE_ARRAY_LIST = "pose_list";
        inline static std::string PUMPKIN_FACE_FRAME = "pumpkin_face_frame";
        inline static std::string TRAJECTORY_OUTPUT = "joint_trajectory";
        inline static std::string NUM_POINTS = "num_points";

        inline static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::vector<geometry_msgs::msg::PoseArray>>(POSE_ARRAY_LIST), 
                                    BT::InputPort<geometry_msgs::msg::Pose>(PUMPKIN_FACE_FRAME),
                                    BT::OutputPort<trajectory_msgs::msg::JointTrajectory>(TRAJECTORY_OUTPUT),
                                    BT::OutputPort<int>(NUM_POINTS)};
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
