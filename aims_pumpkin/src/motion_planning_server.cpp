#include <rclcpp/rclcpp.hpp>
#include <tesseract_rosutils/utils.h>
#include <tesseract_environment/environment_monitor.h>
#include <tesseract_common/macros.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <Eigen/Geometry>

#include <tesseract_scene_graph/graph.h>
#include <tesseract_state_solver/state_solver.h>

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_common/profile_dictionary.h>


#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_graph.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_log.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>

#include <pumpkin_msgs/srv/plan_motion.hpp>

// Topics
static const std::string TESSERACT_MONITOR_NAMESPACE = "pumpkin_environment";

// Services
static const std::string PLANNING_SERVICE = "generate_motion_plan";
static const std::string FREESPACE_PLANNING_SERVICE = "generate_freespace_motion_plan";

static const std::string MOTION_GROUP = "manipulator";
static const std::string TCP_FRAME = "tool0";

class PlanningServer
{
  public:
    PlanningServer(rclcpp::Node::SharedPtr node) 
      : node_(node), env_(std::make_shared<tesseract_environment::Environment>())
    {
      // ROS PARAMETERS
      node_->declare_parameter("robot_description", "");
      node_->declare_parameter("robot_description_semantic", "");

      auto urdf_string = node_->get_parameter("robot_description").as_string();
      auto srdf_string = node_->get_parameter("robot_description_semantic").as_string();

      config = node_->declare_parameter("task_composer_config_file", "aims_pumpkin/config/task_composer_plugins.yaml");
      

      // initial setup
      auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
      if (!env_->init(urdf_string, srdf_string, locator))
        throw std::runtime_error("Failed to initialize environment");
      
      // Create monitor
      auto monitor = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(node_, env_, TESSERACT_MONITOR_NAMESPACE);
      monitor->startPublishingEnvironment();
    
      // Create services
      planning_service_ = node_->create_service<pumpkin_msgs::srv::PlanMotion>(
        PLANNING_SERVICE,
        std::bind(&PlanningServer::planMotionCallback, this, std::placeholders::_1, std::placeholders::_2)
      );
    }

  private:
    rclcpp::Node::SharedPtr node_;
    tesseract_environment::Environment::Ptr env_;
    rclcpp::Service<pumpkin_msgs::srv::PlanMotion>::SharedPtr planning_service_;
    std::string config;


    void planMotionCallback(
      const std::shared_ptr<pumpkin_msgs::srv::PlanMotion::Request> request,
      std::shared_ptr<pumpkin_msgs::srv::PlanMotion::Response> response)
    {
      RCLCPP_INFO(node_->get_logger(), "Received motion planning request");

      try {
        Eigen::VectorXd home_position(6); // Assuming 6-DOF robot
        home_position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        
        tesseract_common::ManipulatorInfo manip_info;
        manip_info.manipulator = MOTION_GROUP;
        manip_info.tcp_frame = TCP_FRAME;
        manip_info.working_frame = env_->getJointGroup(manip_info.manipulator)->getBaseLinkName();

        // Create composite instruction for the complete motion plan
        tesseract_planning::CompositeInstruction composite_instruction("DEFAULT");

        // Freespace move to home position
        tesseract_planning::MoveInstruction start_instruction(
          tesseract_planning::StateWaypoint(env_->getJointGroup(MOTION_GROUP)->getJointNames(), home_position),
          tesseract_planning::MoveInstructionType::FREESPACE
        );
        start_instruction.setManipulatorInfo(manip_info);
        composite_instruction.push_back(start_instruction);

        // Process each PoseArray in the request
        // if there are multiple PoseArrays, a freespace move will be added between them
        int current_idx = 0;
        for (const auto& pose_array : request->path) {
          for (size_t i = 0; i < pose_array.poses.size(); ++i) {
            const auto& pose = pose_array.poses[i];
            
            // Convert geometry_msgs::Pose to Eigen::Isometry3d
            Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
            target_pose.translation().x() = pose.position.x;
            target_pose.translation().y() = pose.position.y;
            target_pose.translation().z() = pose.position.z;
            
            Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, 
                                   pose.orientation.y, pose.orientation.z);
            target_pose.linear() = quat.toRotationMatrix();

            // Create cartesian move instruction
            tesseract_planning::MoveInstruction move_instruction(
              tesseract_planning::CartesianWaypoint(target_pose),
              tesseract_planning::MoveInstructionType::LINEAR);
            move_instruction.setManipulatorInfo(manip_info);
            
            composite_instruction.push_back(move_instruction);
          }
          current_idx++;
          // Add freespace move between different PoseArrays
          if (current_idx < request->path.size()) {

            auto next_pose_array = request->path[current_idx];
            if (next_pose_array.poses.empty())
              continue;
            
            Eigen::Isometry3d next_pose = Eigen::Isometry3d::Identity();
            next_pose.translation().x() = next_pose_array.poses[0].position.x;
            next_pose.translation().y() = next_pose_array.poses[0].position.y;
            next_pose.translation().z() = next_pose_array.poses[0].position.z;
            tesseract_planning::MoveInstruction transit_instruction(
              tesseract_planning::CartesianWaypoint(next_pose),
              tesseract_planning::MoveInstructionType::FREESPACE
            );
            transit_instruction.setManipulatorInfo(manip_info);
            composite_instruction.push_back(transit_instruction);
          }
        }

        // Freespace move back to home position
        tesseract_planning::MoveInstruction end_instruction(
          tesseract_planning::StateWaypoint(env_->getJointGroup(MOTION_GROUP)->getJointNames(), home_position),
          tesseract_planning::MoveInstructionType::FREESPACE);
        end_instruction.setManipulatorInfo(manip_info);
        composite_instruction.push_back(end_instruction);

        tesseract_common::ProfileDictionary::Ptr profile_dict = std::make_shared<tesseract_common::ProfileDictionary>();

        // Setup Task Composer
        tesseract_planning::CompositeInstruction program = composite_instruction;
        auto program_results = plan(program, profile_dict, "motion_planning_task");
        
        // Print the program results for debugging
        RCLCPP_INFO(node_->get_logger(), "Planned %zu instructions", program_results.size());
        
        // set inital state
        response->success = true;
        response->message = "Motion plan generated successfully";
        // TODO: Convert CompositeInstruction to JointTrajectory and set response->trajectory
        
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Motion planning failed: %s", e.what());
        response->success = false;
        response->message = std::string("Motion planning failed: ") + e.what();
      }
    }

  tesseract_planning::CompositeInstruction plan(const tesseract_planning::CompositeInstruction& program,
                                                tesseract_common::ProfileDictionary::Ptr profile_dict,
                                                const std::string& task_name)
  {
    // Set up task composer problem
    auto task_composer_config_file = config;
    const YAML::Node task_composer_config = YAML::LoadFile(task_composer_config_file);
    tesseract_planning::TaskComposerPluginFactory factory(task_composer_config, *env_->getResourceLocator());

    auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");
    tesseract_planning::TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_name);
    if (!task)
      throw std::runtime_error("Failed to create '" + task_name + "' task");

    // Save dot graph
    {
      std::ofstream tc_out_data(tesseract_common::getTempPath() + task_name + ".dot");
      task->dump(tc_out_data);
    }

    auto task_data = std::make_shared<tesseract_planning::TaskComposerDataStorage>();
    task_data->setData("input_program", program);
    task_data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    task_data->setData("profiles", profile_dict);

    // Run problem
    tesseract_planning::TaskComposerLog log(program.getDescription());
    log.initial_data = *task_data;
    tesseract_planning::TaskComposerFuture::UPtr result = executor->run(*task, task_data, true);
    result->wait();
    log.context = result->context;

    // Save the full log, including the output dot graph
    {
      std::stringstream dotgraph_ss;
      static_cast<const tesseract_planning::TaskComposerGraph&>(*task).dump(dotgraph_ss, nullptr,
                                                                            result->context->task_infos.getInfoMap());

      // Save the dot graph to a separate file for convenience
      {
        std::ofstream output_graph(tesseract_common::getTempPath() + task_name + "_results.dot");
        output_graph << dotgraph_ss.str();
      }

      log.dotgraph = dotgraph_ss.str();

      // Save task composer log
      const std::string log_filepath = tesseract_common::getTempPath() + task_name + "_log";
      tesseract_common::Serialization::toArchiveFileBinary<tesseract_planning::TaskComposerLog>(log, log_filepath);
    }

    // Check for successful plan
    if (!result->context->isSuccessful() || result->context->isAborted())
      throw std::runtime_error("Failed to create motion plan");

    // Get results of successful plan
    tesseract_planning::CompositeInstruction program_results =
        result->context->data_storage->getData(task->getOutputKeys().get("program"))
            .as<tesseract_planning::CompositeInstruction>();


    return program_results;
  }
    


};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("motion_planning_server");
  auto server = std::make_shared<PlanningServer>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}