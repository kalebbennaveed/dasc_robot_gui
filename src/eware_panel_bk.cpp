// #include "eware_panel.hpp"

// #include "pluginlib/class_list_macros.hpp"

// // Create two publishers
// // 1. Eware mission status to publish the EwareMissionStatus.msg to signal onboard computer to start the mission
// // 2. (Might not need) Trajectory Visualizer


// namespace dasc_robot_gui {

// using std::placeholders::_1;

// EwarePanel::EwarePanel(QWidget *parent) : rviz_common::Panel(parent) {

//   // construct the layout

//   // topic layout
//   QHBoxLayout *topic_layout = new QHBoxLayout;
//   topic_layout->addWidget(new QLabel("Robot Namespace:"));
//   output_topic_editor_ = new QLineEdit;
//   topic_layout->addWidget(output_topic_editor_);


//   // Layout the grid of the EKF and Target Setpoints
//   QHBoxLayout *pos_layout = new QHBoxLayout;
//   QGridLayout *grid_layout = new QGridLayout;
//   grid_layout->setHorizontalSpacing(3);
//   pos_layout->addItem(grid_layout);

//   // header row
//   // add(*Widget, row, col, rowspan, colspan);
//   grid_layout->addWidget(new QLabel("[FRD]"), 0, 0);
//   grid_layout->addWidget(new QLabel("x [m]"), 0, 1);
//   grid_layout->addWidget(new QLabel("y [m]"), 0, 2);
//   grid_layout->addWidget(new QLabel("z [m]"), 0, 3);
//   grid_layout->addWidget(new QLabel("yaw [°]"), 0, 4);


//   setpoint_pub = new QCheckBox("publish");
//   setpoint_pub->setChecked(false);
//   grid_layout->addWidget(setpoint_x, 1, 1);
//   grid_layout->addWidget(setpoint_y, 1, 2);
//   grid_layout->addWidget(setpoint_z, 1, 3);
//   grid_layout->addWidget(setpoint_yaw, 1, 4);
//   grid_layout->addWidget(setpoint_pub, 1, 5);

//   // setpoint display
//   setpoint_x_disp = new QLabel;
//   setpoint_y_disp = new QLabel;
//   setpoint_z_disp = new QLabel;
//   setpoint_yaw_disp = new QLabel;
//   for (auto s :
//        {setpoint_x_disp, setpoint_y_disp, setpoint_z_disp, setpoint_yaw_disp}) {
//     s->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
//   }

//   grid_layout->addWidget(new QLabel("Setpoint:"), 2, 0);
//   grid_layout->addWidget(setpoint_x_disp, 2, 1);
//   grid_layout->addWidget(setpoint_y_disp, 2, 2);
//   grid_layout->addWidget(setpoint_z_disp, 2, 3);
//   grid_layout->addWidget(setpoint_yaw_disp, 2, 4);


//   // EKF row
//   grid_layout->addWidget(new QLabel("EKF:"), 3, 0);
//   ekf_x = new QLabel("NaN");
//   ekf_y = new QLabel("NaN");
//   ekf_z = new QLabel("NaN");
//   ekf_yaw = new QLabel("NaN");
//   ekf_valid = new QLabel("INVALID");
//   for (auto e : {ekf_x, ekf_y, ekf_z, ekf_yaw, ekf_valid}) {
//     e->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
//   }
//   grid_layout->addWidget(ekf_x, 3, 1);
//   grid_layout->addWidget(ekf_y, 3, 2);
//   grid_layout->addWidget(ekf_z, 3, 3);
//   grid_layout->addWidget(ekf_yaw, 3, 4);
//   grid_layout->addWidget(ekf_valid, 3, 5);


//     // mocap row
//   grid_layout->addWidget(new QLabel("Mocap:"), 4, 0);
//   mocap_x = new QLabel("NaN");
//   mocap_y = new QLabel("NaN");
//   mocap_z = new QLabel("NaN");
//   mocap_yaw = new QLabel("NaN");
//   mocap_valid = new QLabel("Invalid");
//   for (auto m : {mocap_x, mocap_y, mocap_z, mocap_yaw, mocap_valid}) {
//     m->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
//   }
//   grid_layout->addWidget(mocap_x, 4, 1);
//   grid_layout->addWidget(mocap_y, 4, 2);
//   grid_layout->addWidget(mocap_z, 4, 3);
//   grid_layout->addWidget(mocap_yaw, 4, 4);
//   grid_layout->addWidget(mocap_valid, 4, 5);


//   // params row
//   QHBoxLayout *param_layout = new QHBoxLayout;
//   param_name_ = new QLineEdit;
//   param_name_->setPlaceholderText("param name");
//   param_name_->setMaxLength(16);
//   param_get_label_ = new QLabel("[?]");
//   param_get_button_ = new QPushButton("Get");
//   param_set_ = new QLineEdit;
//   param_set_->setPlaceholderText("new value");
//   param_set_button_ = new QPushButton("Set");
//   param_layout->addWidget(new QLabel("Param:"));
//   param_layout->addWidget(param_name_);
//   param_layout->addWidget(param_get_button_);
//   param_layout->addWidget(param_get_label_);
//   param_layout->addWidget(param_set_);
//   param_layout->addWidget(param_set_button_);

//   // Motor Layout
//   QHBoxLayout *raw_motor_layout = new QHBoxLayout;
//   raw_motor_layout->addWidget(new QLabel("Raw Motor CMD:"));
//   motor_num = new QSpinBox;
//   motor_num->setRange(1, 4);
//   raw_motor_layout->addWidget(motor_num);
//   motor_cmd = new QDoubleSpinBox;
//   motor_cmd->setRange(0, 1);
//   motor_cmd->setSingleStep(0.1);
//   raw_motor_layout->addWidget(motor_cmd);
//   raw_mode = new QCheckBox("raw mode?", this);
//   raw_motor_layout->addWidget(raw_mode);

//   // state labels
//   QHBoxLayout *status_layout = new QHBoxLayout;
//   status_label_ = new QLabel("status: UNKNOWN");
//   battery_status_label_ = new QLabel("Bat Status");
//   status_layout->addWidget(status_label_);
//   status_layout->addWidget(battery_status_label_);


//   // create the arm disarm buttons
//   QHBoxLayout *button_layout_1 = new QHBoxLayout;
//   arm_button_ = new QPushButton("Arm", this);
//   offboard_button_ = new QPushButton("Offboard", this);
//   start_button_ = new QPushButton("Start Mission", this);
//   arm_button_->setDisabled(true);
//   offboard_button_->setDisabled(true);
//   start_button_->setDisabled(true);

//   // button_layout_1->addWidget(arm_button_);
//   // button_layout_1->addWidget(offboard_button_);
//   // button_layout_1->addWidget(land_button_);  
//   for (auto b : {arm_button_, offboard_button_, start_button_}){
//     button_layout_1->addWidget(b);
//   }

//     // create the disarm button
//   QHBoxLayout *button_layout_2 = new QHBoxLayout;
//   disarm_button_ = new QPushButton("Disarm", this);
//   land_button_ = new QPushButton("Stop Mission", this);
//   disarm_button_->setDisabled(false);
//   land_button_->setDisabled(true);
//   button_layout_2->addWidget(disarm_button_);
//   button_layout_2->addWidget(land_button_);

//   // Lay out the topic field above the control widget.
//   QVBoxLayout *layout = new QVBoxLayout;
//   layout->addLayout(topic_layout);
//   layout->addLayout(pos_layout);
//   layout->addLayout(param_layout);
//   layout->addLayout(raw_motor_layout);
//   layout->addLayout(status_layout);
//   layout->addLayout(button_layout_1);
//   layout->addLayout(button_layout_2);
//   setLayout(layout);

//   // Create a timer for sending the output.  
//   QTimer *output_timer = new QTimer(this);
//   setpoint_pub_timer_ = new QTimer(this);

//   // Next we make signal/slot connections.
//   connect(output_topic_editor_, SIGNAL(editingFinished()), this,
//           SLOT(updateTopic()));


//   connect(arm_button_, &QPushButton::clicked, this, [this]() {
//     this->commander_set_state(px4_msgs::msg::CommanderSetState::STATE_ARMED);
//     mode = Mode::GROUNDED;
//   });
//   connect(offboard_button_, &QPushButton::clicked, this, [this]() {
//     this->commander_set_state(px4_msgs::msg::CommanderSetState::STATE_OFFBOARD);
//     mode = Mode::HOVERING;
//   });

//   connect(start_button_, &QPushButton::clicked, this, [this]() {
//     mode = Mode::STARTED;
//   });

//   connect(land_button_, &QPushButton::clicked, this, [this]() {
//     this->commander_set_state(px4_msgs::msg::CommanderSetState::STATE_LAND);
//     mode = Mode::STOPPED;
//     setpoint_pub->setChecked(false);
//   });
//   connect(disarm_button_, &QPushButton::clicked, this, [this]() {
//     this->commander_set_state(px4_msgs::msg::CommanderSetState::STATE_DISARMED);
//     mode = Mode::GROUNDED;
//     setpoint_pub->setChecked(false);
//   });
//   connect(param_get_button_, &QPushButton::clicked, this,
//           [this]() { this->parameter_req(false); });
//   connect(param_set_button_, &QPushButton::clicked, this,
//           [this]() { this->parameter_req(true); });

//   // connect the setpoint publisher
//   // connect(setpoint_pub, &QCheckBox::stateChanged, this, [this]() {
//   //   if (this->setpoint_pub->isChecked()) {
//   //     this->setpoint_pub_timer_->start(50);
//   //   } else {
//   //     this->setpoint_pub_timer_->stop();
//   //   }
//   // });

//   connect(output_timer, SIGNAL(timeout()), this, SLOT(timer_callback()));
//   // connect(setpoint_pub_timer_, SIGNAL(timeout()), this,
//   //         SLOT(setpoint_pub_timer_callback()));

//   // Start the main timer.
//   output_timer->start(200); // ms

//   // Create the node
//   node_ = std::make_shared<rclcpp::Node>("eware_panel_node");

// }



// void EwarePanel::updateTopic() {

//   std::cout << "update topic!" << std::endl;

//   setTopic(output_topic_editor_->text());

//   return;
// }

// void EwarePanel::setTopic(const QString &new_string) {

//   // only take action if the name has changed.
//   if (new_string == output_topic_) {
//     return;
//   }

//   output_topic_ = new_string;

//   // Mission status pub
//   if (eware_mission_status_pub_ != NULL) {
//     eware_mission_status_pub_.reset();
//   }

//   // if the pub/sub exists, reset it first
//   if (commander_set_state_pub_ != NULL) {
//     commander_set_state_pub_.reset();
//   }

//   if (trajectory_setpoint_pub_ != NULL) {
//     trajectory_setpoint_pub_.reset();
//   }

//   if (trajectory_setpoint_sub_ != NULL) {
//     trajectory_setpoint_sub_.reset();
//   }

//   if (vehicle_local_pos_sub_ != NULL) {
//     vehicle_local_pos_sub_.reset();
//   }

//   if (commander_status_sub_ != NULL) {
//     commander_status_sub_.reset();
//   }

//   // if (vehicle_visual_odometry_sub_ != NULL) {
//   //   vehicle_visual_odometry_sub_.reset();
//   // }

//   if (parameter_req_pub_ != NULL) {
//     parameter_req_pub_.reset();
//   }

//   if (parameter_res_sub_ != NULL) {
//     parameter_res_sub_.reset();
//   }

//   if (current_setpoint_viz_pub_ != NULL) {
//     current_setpoint_viz_pub_.reset();
//   }

//   reset();

//   // If the topic is the empty string, don't publish anything.
//   if (output_topic_ != "") {

//     // create publishers
//     commander_set_state_pub_ =
//         node_->create_publisher<px4_msgs::msg::CommanderSetState>(
//             output_topic_.toStdString() + "/fmu/in/commander_set_state", 1);

//     trajectory_setpoint_pub_ =
//         node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
//             output_topic_.toStdString() + "/fmu/in/trajectory_setpoint", 1);

//     parameter_req_pub_ = node_->create_publisher<px4_msgs::msg::ParameterReq>(
//         output_topic_.toStdString() + "/fmu/in/parameter_req", 1);

//     current_setpoint_viz_pub_ =
//         node_->create_publisher<geometry_msgs::msg::PoseStamped>(
//             output_topic_.toStdString() + "/viz/trajectory_setpoint", 1);


//     eware_mission_status_pub_ = node_->create_publisher<dasc_msgs::msg::EwareMissionStatus>(
//       output_topic_.toStdString() + "/gs/eware_mission_status", 1);


//     // create subscribers
//     rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
//     auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
//                             qos_profile);
//     auto new_topic =
//         output_topic_.toStdString() + "/fmu/out/vehicle_local_position";
//     std::cout << "Subscribing to: " << new_topic << std::endl;
//     vehicle_local_pos_sub_ =
//         node_->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
//             new_topic, qos,
//             std::bind(&EwarePanel::vehicle_local_pos_cb, this, _1));

//     new_topic = output_topic_.toStdString() + "/fmu/out/commander_status";
//     std::cout << "Subscribing to: " << new_topic << std::endl;
//     commander_status_sub_ =
//         node_->create_subscription<px4_msgs::msg::CommanderStatus>(
//             new_topic, qos,
//             std::bind(&EwarePanel::commander_status_cb, this, _1));

//     new_topic = output_topic_.toStdString() + "/fmu/in/trajectory_setpoint";
//     std::cout << "Subscribing to: " << new_topic << std::endl;
//     trajectory_setpoint_sub_ =
//         node_->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
//             new_topic, qos,
//             std::bind(&EwarePanel::trajectory_setpoint_cb, this, _1));

//     new_topic =
//         output_topic_.toStdString() + "/fmu/in/vehicle_visual_odometry";
//     std::cout << "Subscribing to: " << new_topic << std::endl;
//     vehicle_visual_odometry_sub_ =
//         node_->create_subscription<px4_msgs::msg::VehicleOdometry>(
//             new_topic, qos,
//             std::bind(&EwarePanel::vehicle_visual_odometry_cb, this, _1));


//     new_topic = output_topic_.toStdString() + "/fmu/out/parameter_res";
//     std::cout << "Subscribing to: " << new_topic << std::endl;
//     parameter_res_sub_ =
//         node_->create_subscription<px4_msgs::msg::ParameterRes>(
//             new_topic, qos,
//             std::bind(&EwarePanel::parameter_res_cb, this, _1));

//     new_topic = output_topic_.toStdString() + "/fmu/out/simple_battery_status";
//     std::cout << "Subscribing to: " << new_topic << std::endl;
//     battery_status_sub_ =
//         node_->create_subscription<px4_msgs::msg::SimpleBatteryStatus>(
//             new_topic, qos,
//             std::bind(&EwarePanel::battery_status_cb, this, _1));
//   }

//   // emit config changed signal
//   Q_EMIT configChanged();
// }


// void EwarePanel::setpoint_pub_timer_callback() {

//   if (rclcpp::ok() && trajectory_setpoint_pub_ != NULL) {
//     // construct the setpoint
//     px4_msgs::msg::TrajectorySetpoint msg;
//     if (raw_mode->isChecked()) {
//       msg.raw_mode = true;
//       for (size_t i = 0; i < 4; i++) {
//         msg.cmd[i] = 0;
//       }
//       msg.cmd[motor_num->value() - 1] = motor_cmd->value();
//     } else {
//       msg.raw_mode = false;
//       msg.position[0] = setpoint_x->value();
//       msg.position[1] = setpoint_y->value();
//       msg.position[2] = setpoint_z->value();
//       msg.yaw = (float)(M_PI / 180.f) * (float)(setpoint_yaw->value());
//       for (std::size_t i = 0; i < 3; i++) {
//         msg.velocity[i] = 0;
//         msg.acceleration[i] = 0;
//         msg.jerk[i] = 0;
//       }
//       msg.yawspeed = 0;
//     }
//     trajectory_setpoint_pub_->publish(msg);
//   }
// }


// void EwarePanel::battery_status_cb(
//     const px4_msgs::msg::SimpleBatteryStatus::SharedPtr msg) const {

//   //if (msg->connected) {
//     QString text;
//     text.sprintf("%02.2f%% (%02.1fV, %02.1fA)", 100 * msg->remaining,
//                  msg->voltage_filtered_v, msg->current_filtered_a);
//     battery_status_label_->setText(text);
//   //} else {
//   //  battery_status_label_->setText(QString("Bat Disconnected"));
//   //}
//   return;
// }


// void EwarePanel::parameter_res_cb(
//     const px4_msgs::msg::ParameterRes::SharedPtr msg) const {

//   if (msg->is_int32) {
//     param_get_label_->setText(
//         QString("[int] %1").arg(msg->value, 5, 'f', 0, ' '));
//   } else {
//     param_get_label_->setText(
//         QString("[float] %1").arg(msg->value, 5, 'f', 3, ' '));
//   }
// }

// void EwarePanel::vehicle_visual_odometry_cb(
//     const px4_msgs::msg::VehicleOdometry::SharedPtr msg) const {

//   // check that the frame is NED
//   if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
//     for (auto m : {mocap_x, mocap_y, mocap_z, mocap_yaw}) {
//       m->setText("?");
//     }
//     mocap_valid->setText("Frame NOT NED");
//     mocap_valid->setStyleSheet("QLabel { color : red; }");
//   }

//   mocap_x->setText(QString("%1").arg(msg->position[0], 5, 'f', 3, ' '));
//   mocap_y->setText(QString("%1").arg(msg->position[1], 5, 'f', 3, ' '));
//   mocap_z->setText(QString("%1").arg(msg->position[2], 5, 'f', 3, ' '));

//   tf2::Quaternion q(msg->q[3], msg->q[0], msg->q[1], msg->q[2]);
//   tf2::Matrix3x3 m(q);
//   double roll, pitch, yaw;
//   m.inverse().getEulerYPR(yaw, pitch, roll);
//   // TODO(dev): figure out why we need this weird conversion
//   yaw = M_PI - yaw;

//   // write it to the gui
//   mocap_yaw->setText(
//       QString("%1").arg(float(180.0 / M_PI * yaw), 5, 'f', 1, ' '));

//   mocap_valid->setText("VALID");
//   mocap_valid->setStyleSheet("QLabel { color : green; }");
// }


// void EwarePanel::trajectory_setpoint_cb(
//     const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) const {

//   // update the setpoint display
//   setpoint_x_disp->setText(QString("%1").arg(msg->position[0], 5, 'f', 3, ' '));
//   setpoint_y_disp->setText(QString("%1").arg(msg->position[1], 5, 'f', 3, ' '));
//   setpoint_z_disp->setText(QString("%1").arg(msg->position[2], 5, 'f', 3, ' '));
//   setpoint_yaw_disp->setText(
//       QString("%1").arg(float(180.0 / M_PI) * msg->yaw, 5, 'f', 1, ' '));

//   // also publish visualization
//   geometry_msgs::msg::PoseStamped viz_msg;
//   viz_msg.header.frame_id = "vicon/world";
//   viz_msg.pose.position.x = msg->position[1];
//   viz_msg.pose.position.y = msg->position[0];
//   viz_msg.pose.position.z = -msg->position[2];

//   tf2::Quaternion q;
//   q.setRPY(0, 0, 0.5 * M_PI - msg->yaw);
//   viz_msg.pose.orientation.x = q.x();
//   viz_msg.pose.orientation.y = q.y();
//   viz_msg.pose.orientation.z = q.z();
//   viz_msg.pose.orientation.w = q.w();

//   current_setpoint_viz_pub_->publish(viz_msg);
// }


// void EwarePanel::commander_status_cb(
//     const px4_msgs::msg::CommanderStatus::SharedPtr msg) {

//   last_timestamp_commander_status_ = node_->get_clock()->now().nanoseconds();

//   // ARM
//   arm_button_->setDisabled(msg->state !=
//                            px4_msgs::msg::CommanderStatus::STATE_DISARMED);

//   // Offboard
//   offboard_button_->setDisabled(msg->state !=
//                                 px4_msgs::msg::CommanderStatus::STATE_ARMED);

//   // LAND
//   land_button_->setDisabled(msg->state !=
//                             px4_msgs::msg::CommanderStatus::STATE_OFFBOARD);

//   // DISARM
//   disarm_button_->setDisabled(msg->state ==
//                               px4_msgs::msg::CommanderStatus::STATE_DISARMED);

//   switch (msg->state) {

//   case px4_msgs::msg::CommanderStatus::STATE_DISARMED:
//     status_label_->setText("state: DISARMED");
//     return;
//   case px4_msgs::msg::CommanderStatus::STATE_ARMED:
//     status_label_->setText("state: ARMED");
//     return;
//   case px4_msgs::msg::CommanderStatus::STATE_OFFBOARD:
//     status_label_->setText("state: MISSON");
//     return;
//   case px4_msgs::msg::CommanderStatus::STATE_LAND:
//     status_label_->setText("state: LANDING");
//     return;
//   default:
//     return;
//   }
// }

// float eware_wrapToPi(float yaw)
// {
//   return std::atan2(std::sin(yaw), std::cos(yaw));
// } 

// float eware_wrapTo2Pi(float yaw)
// {

//   yaw = eware_wrapToPi(yaw);
//   if (yaw < 0) {
//     return yaw + 2*M_PI;
//   }

//   return yaw;
// }

// // TODO: Fix this method; make sure nothing is connected to setting setpoint
// void EwarePanel::reset() {

//   for (auto l :
//        {setpoint_x_disp, setpoint_y_disp, setpoint_z_disp, setpoint_yaw_disp}) {
//     l->setText("?");
//   }
//   for (auto l : {ekf_x, ekf_y, ekf_z, ekf_yaw}) {
//     l->setText("?");
//   }
//   ekf_valid->setText("INVALID");
//   ekf_valid->setStyleSheet("QLabel { color : red; }");

//   for (auto l : {mocap_x, mocap_y, mocap_z, mocap_yaw}) {
//     l->setText("?");
//   }
//   mocap_valid->setText("NO MSGS");
//   mocap_valid->setStyleSheet("QLabel {color : red; }");
// }

// void EwarePanel::vehicle_local_pos_cb(
//     const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) const {

//   if (msg->xy_valid && msg->z_valid && msg->v_xy_valid && msg->v_z_valid && msg->heading_good_for_control) {
//     ekf_valid->setText("VALID");
//     ekf_valid->setStyleSheet("QLabel { color : green; }");
//   } else {
//     ekf_valid->setText("INVALID");
//     ekf_valid->setStyleSheet("QLabel { color : red; }");
//   }

//   ekf_x->setText(QString("%1").arg(msg->x, 5, 'f', 3, ' '));
//   ekf_y->setText(QString("%1").arg(msg->y, 5, 'f', 3, ' '));
//   ekf_z->setText(QString("%1").arg(msg->z, 5, 'f', 3, ' '));
//   float yaw = eware_wrapTo2Pi(msg->heading) * float(180.0 / M_PI);
//   ekf_yaw->setText(
//       QString("%1").arg(yaw, 5, 'f', 1, ' '));
// }


// void EwarePanel::commander_set_state(uint8_t new_state) {
//   if (rclcpp::ok() && commander_set_state_pub_ != NULL) {
//     px4_msgs::msg::CommanderSetState msg;
//     msg.new_state = new_state;
//     commander_set_state_pub_->publish(msg);
//   }
// }


// void EwarePanel::timer_callback() {

//   // Unlike rclcpp::spin, rclcpp::spin_some processess only the work available at the moment and then returns immediately
//   rclcpp::spin_some(node_);


//   // check if the connection is still alive
//   rclcpp::Time now_ = node_->get_clock()->now();
//   const uint64_t status_timeout_ns = 1e9;
//   if (now_.nanoseconds() - last_timestamp_commander_status_ >
//       status_timeout_ns) {
//     // set publish to false
//     setpoint_pub->setChecked(false);
//     // set status to comms lost
//     arm_button_->setDisabled(true);
//     offboard_button_->setDisabled(true);
//     land_button_->setDisabled(false);
//     disarm_button_->setDisabled(false);
//     status_label_->setText("state: COMMS_LOST");
//     return;
//   }

//   // If the eware_mission_status_pub_ is NULL, it means that publisher has not been properly initialized 
//   if (!(rclcpp::ok() && eware_mission_status_pub_ != NULL)) {
//     return;
//   }

//   // if pressed grounded (land immediately)
//   if (mode == Mode::GROUNDED || mode == Mode::HOVERING) {
//       eware_mission_status_msg.mission_start = false;
//       eware_mission_status_msg.mission_quit = false;
//       eware_mission_status_pub_->publish(eware_mission_status_msg);  
//     return;
//   }

//   // Start the mission
//   if (mode == Mode::STARTED) {
//     eware_mission_status_msg.mission_start = true;
//     eware_mission_status_msg.mission_quit = false;
//     eware_mission_status_pub_->publish(eware_mission_status_msg);
//     return;
//   }

//   // Stop the mission and land immediately
//   if (mode == Mode::STOPPED) {
//     eware_mission_status_msg.mission_quit = false;
//     eware_mission_status_msg.mission_quit = true;
//     eware_mission_status_pub_->publish(eware_mission_status_msg);
//     return;
//   }
// }

// void EwarePanel::save(rviz_common::Config config) const {

//   rviz_common::Panel::save(config);
//   config.mapSetValue("Topic", output_topic_);
// }


// void EwarePanel::load(const rviz_common::Config &config) {
//   rviz_common::Panel::load(config);

//   QString topic;
//   if (config.mapGetString("Topic", &topic)) {

//     output_topic_editor_->setText(topic);
//     updateTopic();
//   }
// }

// } // namespace dasc_robot_gui

// PLUGINLIB_EXPORT_CLASS(dasc_robot_gui::EwarePanel, rviz_common::Panel)