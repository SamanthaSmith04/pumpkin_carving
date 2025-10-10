#include <rclcpp/rclcpp.hpp>
#include <tesseract_rosutils/utils.h>
#include <tesseract_environment/environment_monitor.h>
#include <tesseract_common/macros.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <Eigen/Geometry>
#include <fstream>

#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/convex_hull_utils.h>
#include <tesseract_collision/vhacd/convex_decomposition_vhacd.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_geometry/mesh_parser.h>
#include <tesseract_geometry/impl/octree_utils.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/command.h>
#include <tesseract_environment/commands.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/conversions.h>
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_graph.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_log.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/planning/profiles/iterative_spline_parameterization_profile.h>
#include <tesseract_task_composer/planning/profiles/min_length_profile.h>

#include <yaml-cpp/yaml.h>

#include <pumpkin_msgs/srv/plan_motion.hpp>
#include <aims_pumpkin/plugins/kinematic_limits_check_profile.h>
#include <aims_pumpkin/profiles.h>

// Additional namespace constants for profiles
static const std::string CONSTANT_TCP_SPEED_TIME_PARAM_TASK_NAME = "ConstantTCPSpeedTimeParameterizationTask";
static const std::string TCP_SPEED_LIMITER_TASK_NAME = "TCPSpeedLimiterTask";
static const std::string SCAN_LINK_NAME = "scan_link";

// Helper function to clamp values
template<typename T>
T clamp(const T& value, const T& min_val, const T& max_val) {
    return std::max(min_val, std::min(value, max_val));
}



// Parameters
//   General
static const std::string VERBOSE_PARAM = "verbose";

//   Task composer
static const std::string TASK_COMPOSER_CONFIG_FILE_PARAM = "task_composer_config_file";
static const std::string RASTER_TASK_NAME_PARAM = "raster_task_name";
static const std::string FREESPACE_TASK_NAME_PARAM = "freespace_task_name";

//   Profile
static const std::string MAX_TRANS_VEL_PARAM = "max_translational_vel";
static const std::string MAX_ROT_VEL_PARAM = "max_rotational_vel";
static const std::string MAX_TRANS_ACC_PARAM = "max_translational_acc";
static const std::string MAX_ROT_ACC_PARAM = "max_rotational_acc";
static const std::string CHECK_JOINT_ACC_PARAM = "check_joint_accelerations";
static const std::string VEL_SCALE_PARAM = "velocity_scaling_factor";
static const std::string ACC_SCALE_PARAM = "acceleration_scaling_factor";
static const std::string LVS_PARAM = "contact_check_lvs_distance";
static const std::string MIN_CONTACT_DIST_PARAM = "min_contact_distance";
static const std::string OMPL_MAX_PLANNING_TIME_PARAM = "ompl_max_planning_time";
static const std::string TCP_MAX_SPEED_PARAM = "tcp_max_speed";
static const std::string TRAJOPT_CARTESIAN_TOLERANCE_PARAM = "cartesian_tolerance";
static const std::string TRAJOPT_CARTESIAN_COEFFICIENT_PARAM = "cartesian_coefficient";

// Topics
static const std::string TESSERACT_MONITOR_NAMESPACE = "pumpkin_environment";

// Services
static const std::string PLANNING_SERVICE = "generate_motion_plan";
static const std::string FREESPACE_PLANNING_SERVICE = "generate_freespace_motion_plan";

static const std::string MOTION_GROUP = "manipulator";
static const std::string TCP_FRAME = "tool0";

static const std::string PROFILE = "pumpkin_profile";


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

      node_->declare_parameter("task_composer_config_file", "");

      config = node_->get_parameter("task_composer_config_file").as_string();

      std::cout << "Task Composer Config File: " << config << std::endl;

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

        std::cout << "Manipulator: " << manip_info.manipulator << std::endl;
        std::cout << "TCP frame: " << manip_info.tcp_frame << std::endl;
        std::cout << "Working frame: " << manip_info.working_frame << std::endl;
        // Create composite instruction for the complete motion plan
        tesseract_planning::CompositeInstruction composite_instruction("DEFAULT");
        
        // Set manipulator info on the composite instruction
        composite_instruction.setManipulatorInfo(manip_info);

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

        tesseract_planning::ProfileDictionary::Ptr profile_dict = std::make_shared<tesseract_planning::ProfileDictionary>();

        updateProfileDictionary(profile_dict);

        std::cout << "Total instructions: " << composite_instruction.size() << std::endl;
        
        // Debug: Print each instruction type
        for (size_t i = 0; i < composite_instruction.size(); ++i) {
          const auto& instruction = composite_instruction[i];
          if (instruction.isMoveInstruction()) {
            const auto& move_inst = instruction.as<tesseract_planning::MoveInstructionPoly>();
            std::string move_type = (move_inst.getMoveType() == tesseract_planning::MoveInstructionType::LINEAR) ? "LINEAR" : "FREESPACE";
            std::cout << "Instruction " << i << ": " << move_type << " move" << std::endl;
          }
        }

        // Setup Task Composer
        auto program_results = plan(composite_instruction, profile_dict, "FreespacePipeline");
        
        // Print the program results for debugging
        RCLCPP_INFO(node_->get_logger(), "Planned %zu instructions", program_results.size());
        
        // Convert the entire program_results to a joint trajectory
        trajectory_msgs::msg::JointTrajectory joint_traj = tesseract_rosutils::toMsg(toJointTrajectory(program_results), env_->getState());
        
        // Set the response
        response->success = true;
        response->message = "Motion plan generated successfully";
        response->trajectory = joint_traj;
        
      } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Motion planning failed: %s", e.what());
        response->success = false;
        response->message = std::string("Motion planning failed: ") + e.what();
      }
    }

  void updateProfileDictionary(tesseract_planning::ProfileDictionary::Ptr profile_dict)
  {    
    double min_contact_dist = 0.01;  // 1cm default
    double longest_valid_segment_length = 0.1;  // 10cm default
    
    // Use default cartesian tolerance vector (6 DOF)
    Eigen::VectorXd cart_tolerance = Eigen::VectorXd::Constant(6, 0.01);  // 1cm tolerance
    Eigen::VectorXd cart_coeff = Eigen::VectorXd::Constant(6, 1.0);  // Unit coefficients
    
    // No collision pairs for now
    std::vector<ExplicitCollisionPair> collision_pairs;
    
    // Default velocity and acceleration scaling factors
    double velocity_scaling_factor = 0.5;
    double acceleration_scaling_factor = 0.5;
    
    // Simple planner
    profile_dict->addProfile(SIMPLE_DEFAULT_NAMESPACE, PROFILE, createSimplePlannerProfile());

    // OMPL
    {
      auto profile = createOMPLProfile(min_contact_dist, collision_pairs, longest_valid_segment_length);
      profile->solver_config.planning_time = 5.0;  // 5 seconds default
      profile_dict->addProfile(OMPL_DEFAULT_NAMESPACE, PROFILE, profile);
    }

    // TrajOpt
    profile_dict->addProfile(TRAJOPT_DEFAULT_NAMESPACE, PROFILE,
                              createTrajOptToolZFreePlanProfile(cart_tolerance, cart_coeff));
    profile_dict->addProfile(TRAJOPT_DEFAULT_NAMESPACE, PROFILE,
                              createTrajOptProfile(min_contact_dist, collision_pairs, longest_valid_segment_length));

    // Descartes
    profile_dict->addProfile(DESCARTES_DEFAULT_NAMESPACE, PROFILE,
                              createDescartesPlanProfile<float>(static_cast<float>(min_contact_dist), collision_pairs,
                                                                longest_valid_segment_length));
    profile_dict->addProfile(DESCARTES_DEFAULT_NAMESPACE, PROFILE, createDescartesSolverProfile<float>());

    // Min length
    profile_dict->addProfile(MIN_LENGTH_DEFAULT_NAMESPACE, PROFILE,
                              std::make_shared<tesseract_planning::MinLengthProfile>(6));

    // ISP profile
    profile_dict->addProfile(ISP_DEFAULT_NAMESPACE, PROFILE,
                              std::make_shared<tesseract_planning::IterativeSplineParameterizationProfile>(
                                  velocity_scaling_factor, acceleration_scaling_factor));

    // // Discrete contact check profile
    // profile_dict->addProfile(
    //     CONTACT_CHECK_DEFAULT_NAMESPACE, PROFILE,
    //     createContactCheckProfile(longest_valid_segment_length, min_contact_dist, collision_pairs));
  

    // Kinematic limit check
    auto check_joint_acc = false;  // Override parameter to disable acceleration checking
    RCLCPP_INFO(node_->get_logger(), "Configuring kinematic limits check profile (acc check: %s)", check_joint_acc ? "true" : "false");
    auto kin_limit_check_profile =
        std::make_shared<aims_pumpkin::KinematicLimitsCheckProfile>(true, true, check_joint_acc);
    profile_dict->addProfile(KINEMATIC_LIMITS_CHECK_TASK_NAME, PROFILE, kin_limit_check_profile);

  }

  tesseract_planning::CompositeInstruction plan(const tesseract_planning::CompositeInstruction& program,
                                                tesseract_planning::ProfileDictionary::Ptr profile_dict,
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

    // Check for successful plan with detailed error logging
    if (!result->context->isSuccessful() || result->context->isAborted()) {
      std::string error_msg = "Motion planning failed: ";
      if (result->context->isAborted()) {
        error_msg += "Task was aborted. ";
      }
      if (!result->context->isSuccessful()) {
        error_msg += "Task was not successful. ";
      }

      // Try to get task info details if available
      try {
        auto task_info_map = result->context->task_infos.getInfoMap();
        for (const auto& task_info_pair : task_info_map) {
          const auto& task_info = task_info_pair.second;
          error_msg += "\nTask '" + task_info.name + "': ";
          error_msg += "return_value=" + std::to_string(task_info.return_value) + " ";
          if (!task_info.status_message.empty()) {
            error_msg += "message='" + task_info.status_message + "' ";
          }
        }
      } catch (...) {
        error_msg += "Unable to retrieve detailed task information. ";
      }
      
      throw std::runtime_error(error_msg);
    }

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