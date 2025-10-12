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
#include <tf2_eigen/tf2_eigen.hpp>

#include <yaml-cpp/yaml.h>

#include <pumpkin_msgs/srv/plan_motion.hpp>
#include <aims_pumpkin/plugins/kinematic_limits_check_profile.h>
#include <aims_pumpkin/plugins/constant_tcp_speed_time_parameterization_profile.h>
#include <aims_pumpkin/profiles.h>

// Additional namespace constants for profiles
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

static const std::string PROFILE = "PumpkinPipeline";


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

    tesseract_common::Toolpath fromMsg(const std::vector<geometry_msgs::msg::PoseArray>& path)
    {
      tesseract_common::Toolpath tps;

      for (const auto& pose_array : path)
      {
        tesseract_common::VectorIsometry3d seg;
        seg.reserve(pose_array.poses.size());

        for (const auto& pose : pose_array.poses)
        {
          Eigen::Isometry3d p;
          tf2::fromMsg(pose, p);
          seg.push_back(p);
        }

        tps.push_back(seg);
      }

      return tps;
    }

    void planMotionCallback(
        const std::shared_ptr<pumpkin_msgs::srv::PlanMotion::Request> request,
        std::shared_ptr<pumpkin_msgs::srv::PlanMotion::Response> response)
    {

// Convert the request path to a Tesseract Toolpath
            tesseract_common::Toolpath raster_strips = fromMsg(request->path);
            RCLCPP_INFO(node_->get_logger(), "Received %zu raster strips", raster_strips.size());

            Eigen::VectorXd home_position(6); // Assuming 6-DOF
            home_position << 0.3, 0.0, 0.0, 0.0, 0.0, 0.0;

            tesseract_common::ManipulatorInfo info;
            info.manipulator = MOTION_GROUP;
            info.tcp_frame = TCP_FRAME;
            info.working_frame = env_->getJointGroup(info.manipulator)->getBaseLinkName();

            std::vector<std::string> joint_names = env_->getJointGroup(info.manipulator)->getJointNames();

            // Profile dictionary
            auto profile_dict = std::make_shared<tesseract_planning::ProfileDictionary>();
            updateProfileDictionary(profile_dict);

        RCLCPP_INFO(node_->get_logger(), "Received motion planning request");
        try {

          std::vector<std::string> joint_names = env_->getJointGroup(info.manipulator)->getJointNames();
          
          tesseract_planning::CompositeInstruction program(PROFILE, info);
          program.setDescription("input_program");
          
          // Define the current state
          tesseract_planning::StateWaypoint current_state(joint_names, env_->getCurrentJointValues(joint_names));
          
          // Add a freespace move from the current state to the first waypoint
              // From start - must be first
              {
                tesseract_planning::CompositeInstruction from_start(PROFILE);
                from_start.setDescription("approach");

                // Define a move to the start waypoint
                from_start.push_back(tesseract_planning::MoveInstruction(
                    current_state, tesseract_planning::MoveInstructionType::FREESPACE, PROFILE, info));

                // Define the target first waypoint
                tesseract_planning::CartesianWaypoint wp1 = raster_strips.at(0).at(0);
                from_start.push_back(
                    tesseract_planning::MoveInstruction(wp1, tesseract_planning::MoveInstructionType::FREESPACE, PROFILE, info));

                // Add the composite to the program
                program.push_back(from_start);
              }
                            
              // Process raster segments and transitions
              for (std::size_t rs = 0; rs < raster_strips.size(); ++rs)
              {
                  // // Add raster segment
                  tesseract_planning::CompositeInstruction raster_segment(PROFILE);
                  raster_segment.setDescription("Raster Index " + std::to_string(rs));                  
                  // Add all waypoints with LINEAR motion type
                  for (int i = 1; i < raster_strips[rs].size(); ++i)
                  {
                      tesseract_planning::CartesianWaypoint cart_wp = raster_strips[rs][i];
                      cart_wp.print("Waypoint " + std::to_string(i) + ": ");

                      raster_segment.push_back(
                          tesseract_planning::MoveInstruction(cart_wp, tesseract_planning::MoveInstructionType::LINEAR, PROFILE, info));
                  }
                  program.push_back(raster_segment);

                  // Add transition if not last segment
                  if (rs < raster_strips.size() - 1)
                  {
                      tesseract_planning::CartesianWaypoint twp = raster_strips[rs + 1].front();

                      tesseract_planning::MoveInstruction transition_instruction1(
                          twp, tesseract_planning::MoveInstructionType::FREESPACE, PROFILE, info);
                      transition_instruction1.setDescription("Transition #" + std::to_string(rs + 1));

                      tesseract_planning::CompositeInstruction transition(PROFILE);
                      transition.setDescription("Transition #" + std::to_string(rs + 1));
                      transition.push_back(transition_instruction1);
                      program.push_back(transition);
                  }
              }
                  
                  // Add a move to home position
                {
                    
                    tesseract_planning::StateWaypoint home_wp(joint_names, home_position);
                    tesseract_planning::CompositeInstruction to_end(PROFILE);
                    to_end.setDescription("to_end");
                    to_end.push_back(tesseract_planning::MoveInstruction(
                        home_wp, tesseract_planning::MoveInstructionType::FREESPACE, PROFILE, info));
                    program.push_back(to_end);
                }
                  
              RCLCPP_INFO(node_->get_logger(), "Constructed program with %zu instructions", program.size());
              for (std::size_t i = 0; i < program.size(); ++i)
              {
                const auto& seg = program.at(i);
                std::cout << "Segment " << i << ": description = " << seg.getDescription() << std::endl;
              }

              // Plan the program
              auto planned_program = plan(program, profile_dict, PROFILE);
              // -------------------------------
              // Convert to joint trajectory
              // -------------------------------
              trajectory_msgs::msg::JointTrajectory joint_traj =
              tesseract_rosutils::toMsg(toJointTrajectory(planned_program), env_->getState());
              
              response->success = true;
              response->message = "Motion plan generated successfully";
              response->trajectory = joint_traj;
              RCLCPP_INFO(node_->get_logger(), "Motion plan generated successfully with %zu points", joint_traj.points.size());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Motion planning failed: %s", e.what());
            response->success = false;
            response->message = std::string("Motion planning failed: ") + e.what();
        }
    }


  void updateProfileDictionary(tesseract_planning::ProfileDictionary::Ptr profile_dict)
  {    
    double min_contact_dist = 0.00;
    double longest_valid_segment_length = 0.01;
    
    // Use tighter cartesian tolerance vector (6 DOF)
    Eigen::VectorXd cart_tolerance(6);
    cart_tolerance << 0.000, 0.000, 0.000, 0.050, 0.050, 6.280; 
    Eigen::VectorXd cart_coeff(6);
    cart_coeff << 10.0, 10.0, 10.0, 2.5, 2.5, 0.0; 
    
    // No collision pairs for now
    std::vector<ExplicitCollisionPair> collision_pairs;
    
    // Default velocity and acceleration scaling factors
    double velocity_scaling_factor = 1.0;
    double acceleration_scaling_factor = 1.0;

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

      // Constant TCP time parameterization profile
      double vel_trans = 0.050;
      double vel_rot = 1.571;
      double acc_trans = 0.100;
      double acc_rot = 3.14;
      auto cart_time_param_profile = std::make_shared<snp_motion_planning::ConstantTCPSpeedTimeParameterizationProfile>(
          vel_trans, vel_rot, acc_trans, acc_rot, velocity_scaling_factor, acceleration_scaling_factor);
      profile_dict->addProfile(CONSTANT_TCP_SPEED_TIME_PARAM_TASK_NAME, PROFILE, cart_time_param_profile);

    // // Discrete contact check profile
    profile_dict->addProfile(
        CONTACT_CHECK_DEFAULT_NAMESPACE, PROFILE,
        createContactCheckProfile(longest_valid_segment_length, min_contact_dist, collision_pairs));
  

    // Kinematic limit check
    bool check_position = true;     // Always check position limits
    bool check_velocity = false;    // Only check velocity if time parameterization is enabled
    bool check_acceleration = false; // Only check acceleration if time parameterization is enabled
    RCLCPP_INFO(node_->get_logger(), "Configuring kinematic limits check profile (pos: %s, vel: %s, acc: %s)",
                check_position ? "true" : "false",
                check_velocity ? "true" : "false",
                check_acceleration ? "true" : "false");
    auto kin_limit_check_profile =
        std::make_shared<aims_pumpkin::KinematicLimitsCheckProfile>(check_position, check_velocity, check_acceleration);
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

    // Save task composer log
    const std::string log_filepath = tesseract_common::getTempPath() + task_name + "_log";
    tesseract_common::Serialization::toArchiveFileBinary<tesseract_planning::TaskComposerLog>(log, log_filepath);

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