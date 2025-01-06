// Flow
// Number of robots: Add the number of the robots to be added
// Robot 1 name, Robot 2 name
// Robot 3 name


// Logic
// 1. First fill in the names of the robot 
// 2. Then, enter the number of the robot to initialize mission for
// 3. Press the initialize mission. This create separates topic and set the stage ready and set the mode to topics_initialized
// 4. If topics initialized, keep on checking for commander status
// 5. (Callback) if the commander status is offboard for all the topics then enable the mission button
// 6. (Callback) if the commander status is moved to not in offboard then land all robots (Not required to press the land all)
// 7. (Callback) if land all pressed then move all the robots to the landing mode
// 8. Also allow comman disarming 





#ifndef MESCH_PANEL_HPP_
#define MESCH_PANEL_HPP_

#ifndef Q_MOC_RUN
#include <memory>
#endif

#include "geometry_msgs/msg/twist.hpp"
#include "px4_msgs/msg/simple_battery_status.hpp"
#include "px4_msgs/msg/commander_set_state.hpp"
#include "px4_msgs/msg/commander_status.hpp"
#include "px4_msgs/msg/parameter_req.hpp"
#include "px4_msgs/msg/parameter_res.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

// ros messages 
#include "dasc_msgs/msg/mesch_mission_status.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rviz_common/panel.hpp"
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QTimer>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace dasc_robot_gui {

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz_common::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz_common::Panel.
class meSchPanel : public rviz_common::Panel {
    // This class uses Qt slots and is a subclass of QObject, so it needs
    // the Q_OBJECT macro.
    Q_OBJECT

public:
    // QWidget subclass constructors usually take a parent widget
    // parameter (which usually defaults to 0).  At the same time,
    // pluginlib::ClassLoader creates instances by calling the default
    // constructor (with no arguments).  Taking the parameter and giving
    // a default of 0 lets the default constructor work and also lets
    // someone using the class for something else to pass in a parent
    // widget as they normally would with Qt.
    explicit meSchPanel(QWidget *parent = 0);

    // Now we declare overrides of rviz_common::Panel functions for saving and
    // loading data from the config file.  Here the data is the
    // topic name.
    virtual void load(const rviz_common::Config &config);
    virtual void save(rviz_common::Config config) const;

    // Next come a couple of public Qt slots.

public Q_SLOTS:

    // In this example setTopic() does not get connected to any signal
    // (it is called directly), but it is easy to define it as a public
    // slot instead of a private function in case it would be useful to
    // some other user.
    void setTopic(const QString &topic);

    // Here we declare some internal slots.

protected Q_SLOTS:
    void commander_set_state(uint8_t new_state);

    // updateTopic() reads the topic name from the QLineEdit and calls
    // setTopic() with the result.
    void updateTopic();

    void timer_callback();
    void setpoint_pub_timer_callback();
    void parameter_req(bool set);

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

    // Then we finish up with protected member variables.

protected:
    // One-line text editor for entering the outgoing ROS topic name.
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
    QPushButton *arm_button_, *start_button_, *offboard_button_, *land_button_, *disarm_button_, *start_sim_button_, *stop_sim_button_;

    // Param:
    QLineEdit *param_name_, *param_set_;
    QLabel *param_get_label_;
    QPushButton *param_get_button_, *param_set_button_;

    // Raw mode cmd:
    QSpinBox *motor_num;
    QDoubleSpinBox *motor_cmd;
    QCheckBox *raw_mode;

    // The current name of the output topic.
    QString robot_num_, robot_1_name_, robot_2_name_, robot_3_name_;
    QVector<QString> robot_names_;

    // The ROS node and publisher for the command velocity.
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Publisher<dasc_msgs::msg::MeschMissionStatus>::SharedPtr meSch_mission_status_pub_;
    rclcpp::Publisher<px4_msgs::msg::CommanderSetState>::SharedPtr commander_set_state_pub_; // Shared variable for defining this pub
    rclcpp::Subscription<px4_msgs::msg::CommanderStatus>::SharedPtr commander_status_sub_;    // Shared variable for defining this sub
    std::vector<rclcpp::Publisher<px4_msgs::msg::CommanderSetState>::SharedPtr> commander_set_state_pub_vec_;
    std::vector<rclcpp::Publisher<px4_msgs::msg::CommanderStatus>::SharedPtr> commander_status_sub_vec_;

        
    uint64_t last_timestamp_commander_status_ = 0;
    int robot_num_int_;
    uint8_t robot_1_commander_status_, robot_2_commander_status_, robot_3_commander_status_;

    enum Mode {ALL_GROUNDED, TOPICS_INIT, ALL_OFFBOARD, ALL_STOPPED, STARTED, SIMSTART};

    Mode mode = ALL_GROUNDED;
    dasc_msgs::msg::meSchMissionStatus meSch_mission_status_msg;
};

} // end namespace dasc_robot_gui

#endif // MESCH_PANEL_HPP_
