#pragma once

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/node.hpp"
#include "QWidget"


namespace Ui {
  class PumpkinGUI;
}
namespace pumpkin_widget 
{

class PumpkinGUI : public QWidget
{

  Q_OBJECT

  public:
    explicit PumpkinGUI(rclcpp::Node::SharedPtr ros_node, QWidget *parent = nullptr);

    ~PumpkinGUI();

  private:
    rclcpp::Node::SharedPtr ros_node_;
    Ui::PumpkinGUI* ui_;
    BT::Tree tree;
    BT::Blackboard::Ptr blackboard;
};

} // namespace pumpkin_widget