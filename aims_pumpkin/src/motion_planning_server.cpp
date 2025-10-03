#include <rclcpp/rclcpp.hpp>
#include <tesseract_common/macros.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_environment/core/environment.h>

#include <tesseract_scene_graph/graph.h>
#include <tesseract_state_solver/state_solver.h>

#include <tesseract_kinematics/core/kinematics.h>
#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_motion_planners/simple/simple_planner.h>

// Topics
static const std::string TESSERACT_MONITOR_NAMESPACE = "pumpkin_environment";

// Services
static const std::string PLANNING_SERVICE = "generate_motion_plan";
static const std::string FREESPACE_PLANNING_SERVICE = "generate_freespace_motion_plan";

class PlanningServer
{
  public:
    PlanningServer(rclcpp::Node::SharedPtr node) 
      : node_(node), env_(std::make_shared<tesseract_environment::Environment>())
    {
      // ROS PARAMETERS
      auto urdf_string = node_->declare_parameter("robot_description", "");
      auto srdf_string = node_->declare_parameter("robot_description_semantic", "");

      node_->declare_parameter("task_composer_config_file", "");
      

      // initial setup
      auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
      if (!env_->init(urdf_string, srdf_string, locator))
        throw std::runtime_error("Failed to initialize environment");
      
      // Create monitor
      auto monitor = std::make_shared<tesseract_environment::ROSEnvironmentMonitor>(node, env_, TESSERACT_MONITOR_NAMESPACE);
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


    planMotionCallback(
      const std::shared_ptr<pumpkin_msgs::srv::PlanMotion::Request> request,
      std::shared_ptr<pumpkin_msgs::srv::PlanMotion::Response> response)
    {
      RCLCPP_INFO(node_->get_logger(), "Received motion planning request");

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