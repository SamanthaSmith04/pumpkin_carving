#include "pumpkin_widget.hpp"

#include <action_nodes.hpp>
#include <condition_nodes.hpp>
#include <bt_ros_nodes.hpp>
#include "bt_nodes/bt_nodes_pumpkin.hpp"
#include <QAbstractButton>
#include <QTimer>
#include "ui_pumpkin.h" // will always be "ui_" + <ui file name> + ".h"

namespace pumpkin_widget
{

PumpkinGUI::PumpkinGUI(rclcpp::Node::SharedPtr ros_node, QWidget *parent)
    : QWidget(parent), ros_node_(ros_node), ui_(new Ui::PumpkinGUI)
{

  // find the file for the behavior tree configuration
  std::string package_path = ament_index_cpp::get_package_share_directory("pumpkin_bt_application");
  std::string gui_bt_file = package_path + "/config/aims_pumpkin_bt.xml";  

  // initialize the UI for the GUI
  ui_->setupUi(this);

  blackboard = BT::Blackboard::create();

  // Set blackboard variables
  blackboard->set("main_gui", static_cast<QWidget*>(this));
  blackboard->set("motion_exec_push_button", static_cast<QAbstractButton*>(ui_->motion_exec_push_button));
  blackboard->set("load_traj_from_yaml_push_button", static_cast<QAbstractButton*>(ui_->load_traj_from_yaml_push_button));
  blackboard->set("load_poses_from_file_push_button", static_cast<QAbstractButton*>(ui_->load_poses_from_file_push_button));
  blackboard->set("messages_text_edit", static_cast<QTextEdit*>(ui_->messages_text_edit));
  blackboard->set("project_folder", std::string("/.aims/pumpkin/")); // to be set when a project is loaded

  BT::BehaviorTreeFactory factory;

  RCLCPP_INFO(ros_node_->get_logger(), "Behavior Tree created from file: %s", gui_bt_file.c_str());
  
  // Connect parameters to the tree
  factory.registerNodeType<bt_common_nodes::WaitForGuiInput>("WaitForGuiInput");
  factory.registerNodeType<bt_common_nodes::WaitForGuiInput>("WaitForGuiInputOnTick");
  factory.registerNodeType<bt_common_nodes::OpenFileDialog>("OpenFileDialog");
  factory.registerNodeType<bt_common_nodes::LoadMotionPlanFromYAML>("LoadMotionPlanFromYAML");
  factory.registerNodeType<bt_common_nodes::AddMsgToTextEdit>("AddMsgToTextEdit");
  factory.registerNodeType<bt_common_nodes::ProcessTraj>("ProcessTraj");
  factory.registerNodeType<bt_pumkin_nodes::LoadPosesListFromYAML>("LoadPosesListFromYAML");
  
  // link ros topics, services, and actions
  auto pub_traj_params = BT::RosNodeParams(ros_node_, "trajectory");
  factory.registerNodeType<bt_common_nodes::PubTrajectoryPreview>("PubTrajectoryPreview", pub_traj_params);
  auto start_point_queue_params = BT::RosNodeParams(ros_node_, "start_point_queue_mode");
  factory.registerNodeType<bt_common_nodes::StartPointQueueMode>("StartPointQueueMode", start_point_queue_params);
  auto queue_traj_params = BT::RosNodeParams(ros_node_, "queue_traj_point");
  factory.registerNodeType<bt_common_nodes::QueueTrajPoint>("QueueTrajPoint", queue_traj_params);
  auto joint_state_params = BT::RosNodeParams(ros_node_, "/phantom/joint_states");
  factory.registerNodeType<bt_common_nodes::CheckAtDestination>("CheckAtDestination", joint_state_params);
  
  auto generate_motion_plan_params = BT::RosNodeParams(ros_node_, "generate_motion_plan");
  factory.registerNodeType<bt_pumkin_nodes::GenerateMotionPlan>("GenerateMotionPlan", generate_motion_plan_params);

  auto publish_pose_array_params = BT::RosNodeParams(ros_node_, "loaded_pose_array");
  factory.registerNodeType<bt_pumkin_nodes::PubPoseArrayPreview>("PubPoseArrayPreview", publish_pose_array_params);

  tree = factory.createTreeFromFile(gui_bt_file, blackboard);

  // Start ticking BehaviorTree
  QTimer *tree_timer = new QTimer(this);
  QObject::connect(tree_timer, &QTimer::timeout, [this]() {
      BT::NodeStatus status = this->tree.tickOnce();
  });
  tree_timer->start(10); // Tick every 10 ms

}

PumpkinGUI::~PumpkinGUI()
{
  delete ui_;
}

} // namespace pumpkin_widget