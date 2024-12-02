#ifndef EWARE_PANEL_HPP_
#define EWARE_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <memory>
#endif

// ros
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include <tf2/LinearMath/Quaternion.h>

// px4 ros msgs
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/simple_battery_status.hpp"
#include "px4_msgs/msg/commander_set_state.hpp"
#include "px4_msgs/msg/commander_status.hpp"
#include "px4_msgs/msg/parameter_req.hpp"
#include "px4_msgs/msg/parameter_res.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"


// custom ros messages
#include "dasc_msgs/msg/eware_mission_status.hpp"

// rviz
#include "rviz_common/panel.hpp"

// QT
#include <QButtonGroup>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <stdio.h>
#include <QPainter>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// trajectory
// #include "Lissajous.hpp"

namespace dasc_robot_gui {

class EwarePanel : public rviz_common::Panel {

  Q_OBJECT

public:
  explicit EwarePanel(QWidget *parent = 0);

  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
  void setTopic(const QString &topic);

protected Q_SLOTS:
  void timer_callback();
  void setpoint_pub_timer_callback();
  // void viz_timer_callback();
  void updateTopic();
  void parameter_req(bool set);
  void commander_set_state(uint8_t new_state);
  void trajectory_setpoint_cb(
      const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) const; 
  void vehicle_local_pos_cb(
      const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const;

  void commander_status_cb(const px4_msgs::msg::CommanderStatus::SharedPtr msg);

  void vehicle_visual_odometry_cb(
      const px4_msgs::msg::VehicleOdometry::SharedPtr msg) const;

  void parameter_res_cb(const px4_msgs::msg::ParameterRes::SharedPtr msg) const;

  void
  battery_status_cb(const px4_msgs::msg::SimpleBatteryStatus::SharedPtr msg) const;

  void reset();

protected:
  // ROS Topic name:
  QLineEdit *output_topic_editor_;
  QLabel *battery_status_label_;

  // Setpoint
  QDoubleSpinBox *setpoint_x, *setpoint_y, *setpoint_z, *setpoint_yaw;
  QLabel *setpoint_x_disp, *setpoint_y_disp, *setpoint_z_disp,
      *setpoint_yaw_disp;
  QCheckBox *setpoint_pub;
  QTimer *setpoint_pub_timer_;

    // EKF:
  QLabel *ekf_x, *ekf_y, *ekf_z, *ekf_yaw, *ekf_valid;

  // Mocap:
  QLabel *mocap_x, *mocap_y, *mocap_z, *mocap_yaw, *mocap_valid;

  // Status:
  QLabel *status_label_;
  QPushButton *arm_button_, *offboard_button_, *land_button_, *disarm_button_;

  // Param:
  QLineEdit *param_name_, *param_set_;
  QLabel *param_get_label_;
  QPushButton *param_get_button_, *param_set_button_;

  // Raw mode cmd:
  QSpinBox *motor_num;
  QDoubleSpinBox *motor_cmd;
  QCheckBox *raw_mode;

  // The current name of the output topic.
  QString output_topic_;

  // ROS
  // Node
  std::shared_ptr<rclcpp::Node> node_;

  // Publishers
  rclcpp::Publisher<dasc_msgs::msg::EwareMissionStatus>::SharedPtr eware_mission_status_pub_;
  rclcpp::Publisher<px4_msgs::msg::CommanderSetState>::SharedPtr
      commander_set_state_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::ParameterReq>::SharedPtr parameter_req_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      current_setpoint_viz_pub_;

  // Subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      vehicle_local_pos_sub_;
  rclcpp::Subscription<px4_msgs::msg::CommanderStatus>::SharedPtr
      commander_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
      vehicle_visual_odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      trajectory_setpoint_sub_;
  rclcpp::Subscription<px4_msgs::msg::ParameterRes>::SharedPtr
      parameter_res_sub_;
  rclcpp::Subscription<px4_msgs::msg::SimpleBatteryStatus>::SharedPtr
      battery_status_sub_;

  enum Mode {GROUNDED, STARTED, STOPPED};

  Mode mode = GROUNDED;

  dasc_msgs::msg::EwareMissionStatus eware_mission_status_msg;
  uint64_t last_timestamp_commander_status_ = 0;

}; // class TrajectoryPanel

} // end namespace dasc_robot_gui

#endif // TRAJECTORY_PANEL_HPP_
