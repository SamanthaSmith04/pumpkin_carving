/// Software License Agreement (Apache 2.0 License)
///
/// Copyright (c) 2025, The Ohio State University
/// Center for Design and Manufacturing Excellence (CDME)
/// The Artificially Intelligent Manufacturing Systems Lab (AIMS)
/// All rights reserved.
///
/// Author: Samantha Smith
///

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <QWidget>
#include <QVBoxLayout>
#include <QCloseEvent>

#include "pumpkin_widget.hpp"

namespace pumpkin_widget
{
class PumpkinPanel : public rviz_common::Panel
{
    public:
        void onInitialize() override {
          auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
          auto layout = new QVBoxLayout();
          widget = new PumpkinGUI(node,this);
          layout->addWidget(widget);
          setLayout(layout);
        }

        /**
         * @brief Called when the panel is closed to kill all running panel nodes.
        */
        void closeEvent(QCloseEvent *event) override {
            rclcpp::shutdown();
            rviz_common::Panel::closeEvent(event);
        }

    private:
        PumpkinGUI* widget;
};
}
PLUGINLIB_EXPORT_CLASS(pumpkin_widget::PumpkinPanel, rviz_common::Panel)