#ifndef GBPLANNER_UI_H
#define GBPLANNER_UI_H

#include <string>

#ifndef Q_MOC_RUN
#include <planner_msgs/srv/pci_global.hpp>
#include <planner_msgs/srv/pci_initialization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#endif

class QLineEdit;
class QPushButton;

namespace gbplanner_ui {
class gbplanner_panel : public rviz_common::Panel {
  Q_OBJECT
 public:
  explicit gbplanner_panel(QWidget* parent = nullptr);
  void onInitialize() override;
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

 public Q_SLOTS:
  void on_start_planner_click();
  void on_start_planner_single_click();
  void on_stop_planner_click();
  void on_homing_click();
  void on_init_motion_click();
  void on_plan_to_waypoint_click();
  void on_global_planner_click();
  void on_change_operation_mode_click();
  void on_namespace_changed();

 protected:
  // (Re)create the clients under the robot namespace, e.g. /SpaceUavDrone1.
  void createClients();
  // Fire and forget: the GUI thread never waits for the response.
  template <typename SrvT>
  void callAsync(const typename rclcpp::Client<SrvT>::SharedPtr& client,
                 std::shared_ptr<typename SrvT::Request> request);

  QLineEdit* namespace_line_edit;
  QPushButton* button_start_planner;
  QPushButton* button_start_planner_single;
  QPushButton* button_stop_planner;
  QPushButton* button_homing;
  QPushButton* button_init_motion;
  QPushButton* button_plan_to_waypoint;
  QPushButton* button_global_planner;
  QLineEdit* global_id_line_edit;
  QPushButton* button_change_operation_mode;
  bool waypoint_nav_mode = false;

  rclcpp::Node::SharedPtr node_;
  std::string robot_ns_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr planner_client_start_planner;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr planner_client_start_planner_single;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr planner_client_stop_planner;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr planner_client_homing;
  rclcpp::Client<planner_msgs::srv::PciInitialization>::SharedPtr planner_client_init_motion;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr planner_client_plan_to_waypoint;
  rclcpp::Client<planner_msgs::srv::PciGlobal>::SharedPtr planner_client_global_planner;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr change_operation_mode_client;
};

}  // namespace gbplanner_ui

#endif  // GBPLANNER_UI_H
