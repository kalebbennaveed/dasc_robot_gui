#include "eware_panel.hpp"

#include "pluginlib/class_list_macros.hpp"

// Things to test out
// 1. Increasing frequency for the visualization. Keep the frequency high to see how trajectories can be


namespace dasc_robot_gui {

using std::placeholders::_1;

EwarePanel::EwarePanel(QWidget *parent) : rviz_common::Panel(parent) {

  // construct the layout

  // topic layout
  QHBoxLayout *topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Robot Namespace:"));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget(output_topic_editor_);

  // standard trajectories
  QHBoxLayout *standard_traj_layout = new QHBoxLayout;
  set_hover_button_ = new QPushButton("Hover Traj", this);
  set_circle_button_ = new QPushButton("Circle Traj", this);
  set_lissa_button_ = new QPushButton("Lissajous Traj", this);

  for (auto b : {set_hover_button_, set_circle_button_,
                 set_lissa_button_}) {
    standard_traj_layout->addWidget(b);
  }

  // lissajous
  // QHBoxLayout *lissa_params_layout = new QHBoxLayout;
  QGridLayout *grid_layout = new QGridLayout;
  grid_layout->setHorizontalSpacing(3);
  // lissa_params_layout->addItem(grid_layout);

  // header row
  grid_layout->addWidget(new QLabel("[FRD]"), 0, 0);
  grid_layout->addWidget(new QLabel("x [m]"), 0, 1);
  grid_layout->addWidget(new QLabel("y [m]"), 0, 2);
  grid_layout->addWidget(new QLabel("z [m]"), 0, 3);
  grid_layout->addWidget(new QLabel("yaw [°]"), 0, 4);

  // amplitude
  grid_layout->addWidget(new QLabel("Amplitude [m, °]:"), 1, 0);
  amplitude_x = new QDoubleSpinBox;
  amplitude_y = new QDoubleSpinBox;
  amplitude_z = new QDoubleSpinBox;
  amplitude_yaw = new QDoubleSpinBox;
  int index = 1;
  for (auto s : {amplitude_x, amplitude_y, amplitude_z, amplitude_yaw}) {
    s->setSingleStep(0.1);
    s->setValue(0.0);
    s->setRange(-1000.0, 1000.0);
    s->setWrapping(false);
    grid_layout->addWidget(s, 1, index);
    index++;
  }
  amplitude_yaw->setSingleStep(5.0);

  // freq
  grid_layout->addWidget(new QLabel("Freq [1/s]:"), 2, 0);
  freq_x = new QDoubleSpinBox;
  freq_y = new QDoubleSpinBox;
  freq_z = new QDoubleSpinBox;
  freq_yaw = new QDoubleSpinBox;
  index = 1;
  for (auto s : {freq_x, freq_y, freq_z, freq_yaw}) {
    s->setSingleStep(0.001);
    s->setDecimals(3);
    s->setValue(0.03);
    s->setRange(0.0, 1.0);
    s->setWrapping(false);
    grid_layout->addWidget(s, 2, index);
    index++;
  }

  // phi
  grid_layout->addWidget(new QLabel("Phase [°]:"), 3, 0);
  phi_x = new QDoubleSpinBox;
  phi_y = new QDoubleSpinBox;
  phi_z = new QDoubleSpinBox;
  phi_yaw = new QDoubleSpinBox;
  index = 1;
  for (auto s : {phi_x, phi_y, phi_z, phi_yaw}) {
    s->setSingleStep(5.0);
    s->setValue(0.0);
    s->setRange(0.0, 360.0);
    s->setWrapping(true);
    grid_layout->addWidget(s, 3, index);
    index++;
  }

  // offset
  grid_layout->addWidget(new QLabel("Offset [m, °]:"), 4, 0);
  offset_x = new QDoubleSpinBox;
  offset_y = new QDoubleSpinBox;
  offset_z = new QDoubleSpinBox;
  offset_yaw = new QDoubleSpinBox;
  index = 1;
  for (auto s : {offset_x, offset_y, offset_z, offset_yaw}) {
    s->setSingleStep(0.1);
    s->setValue(0.0);
    s->setRange(-1000.0, 1000.0);
    s->setWrapping(false);
    grid_layout->addWidget(s, 4, index);
    index++;
  }
  offset_yaw->setValue(90.0);

  // buttons
  QHBoxLayout *button_layout = new QHBoxLayout;
  takeoff = new QPushButton("Load Mission", this);
  start = new QPushButton("Start Mission", this);
  hover = new QPushButton("Hover", this);
  stop = new QPushButton("Stop", this);

  for (auto b : {takeoff, start, hover, stop}) {
    button_layout->addWidget(b);
  }

  // stack it all together
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  // layout->addLayout(standard_traj_layout);
  // layout->addLayout(lissa_params_layout);
  layout->addLayout(button_layout);
  setLayout(layout);

  // create the timer
  QTimer *output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(timer_callback()));

  QTimer *viz_timer = new QTimer(this);
  connect(viz_timer, SIGNAL(timeout()), this, SLOT(viz_timer_callback()));

  // connect the buttons

  // trajectory buttons
  connect(set_hover_button_, &QPushButton::clicked, this,
          [this]() { set_traj_hover(); });
  connect(set_circle_button_, &QPushButton::clicked, this,
          [this]() { set_traj_circle(); });
  connect(set_lissa_button_, &QPushButton::clicked, this,
          [this]() { set_traj_lissa(); });

  // mode buttons
  // Takeoff is the go to stat buttom 
  connect(takeoff, &QPushButton::clicked, this, [this]() {
    update_lissa();
    mode = Mode::TAKEOFF;
  });

  connect(start, &QPushButton::clicked, this, [this]() {
    update_lissa();
    mode = Mode::STARTED;
  });
  connect(hover, &QPushButton::clicked, this, [this]() { mode = Mode::HOVER; });
  connect(stop, &QPushButton::clicked, this,
          [this]() { mode = Mode::STOPPED; });

  // connect the slots
  connect(output_topic_editor_, SIGNAL(editingFinished()), this,
          SLOT(updateTopic()));

  // create the node
  node_ = std::make_shared<rclcpp::Node>("eware_panel_node");

  // start the main timer
  output_timer->start(10); // ms
  viz_timer->start(10);   // ms
}

void EwarePanel::set_traj_hover() {

  // amplitude
  for (auto a : {amplitude_x, amplitude_y, amplitude_z, amplitude_yaw}) {
    a->setValue(0.0);
  }

  // freq
  for (auto f : {freq_x, freq_y, freq_z, freq_yaw}) {
    f->setValue(0.03);
  }

  // phase
  for (auto f : {phi_x, phi_y, phi_z, phi_yaw}) {
    f->setValue(0.0);
  }

  // offset
  for (auto o : {offset_x, offset_y, offset_z, offset_yaw}) {
    o->setValue(0.0);
  }
  offset_z->setValue(-1.0);
  offset_yaw->setValue(90.0);
}

void EwarePanel::set_traj_circle() {

  // amplitude
  amplitude_x->setValue(1.0);
  amplitude_y->setValue(1.0);
  for (auto a : {amplitude_z, amplitude_yaw}) {
    a->setValue(0.0);
  }
  amplitude_yaw->setValue(180.0);

  // freq
  for (auto f : {freq_x, freq_y, freq_z, freq_yaw}) {
    f->setValue(0.03);
  }

  // phase
  for (auto f : {phi_x, phi_z, phi_yaw}) {
    f->setValue(0.0);
  }
  phi_y->setValue(90.0);

  // offset
  for (auto o : {offset_x, offset_y, offset_yaw}) {
    o->setValue(0.0);
  }
  offset_z->setValue(-1.0);
  offset_yaw->setValue(90.0);
}

void EwarePanel::set_traj_lissa() {

  // amplitude
  amplitude_x->setValue(1.0);
  amplitude_y->setValue(1.0);
  amplitude_z->setValue(0.2);
  amplitude_yaw->setValue(0.0);

  // freq
  freq_x->setValue(0.03);
  freq_y->setValue(5.0 / 4.0 * 0.03);
  freq_z->setValue(4.0 / 3.0 * 0.03);
  freq_yaw->setValue(0.03);

  // phase
  phi_x->setValue(0.0);
  phi_y->setValue(90.0);
  phi_z->setValue(45.0);
  phi_yaw->setValue(0.0);

  // offset
  for (auto o : {offset_x, offset_y, offset_yaw}) {
    o->setValue(0.0);
  }
  offset_z->setValue(-1.0);
  offset_yaw->setValue(90.0);
}

void EwarePanel::update_lissa() {

  double amplitude[4] = {amplitude_x->value(), amplitude_y->value(),
                         amplitude_z->value(),
                         amplitude_yaw->value() * M_PI / 180.0f};

  double omega[4] = {2 * M_PI * freq_x->value(), 2 * M_PI * freq_y->value(),
                     2 * M_PI * freq_z->value(), 2 * M_PI * freq_yaw->value()};

  double phi[4] = {
      M_PI / 180.0f * phi_x->value(), M_PI / 180.0f * phi_y->value(),
      M_PI / 180.0f * phi_z->value(), M_PI / 180.0f * phi_yaw->value()};

  double offset[4] = {offset_x->value(), offset_y->value(), offset_z->value(),
                      M_PI / 180.0f * offset_yaw->value()};

  lissa.set_params(amplitude, omega, phi, offset);

  std::cout << "params updated" << std::endl;

  traj_start_time = node_->get_clock()->now().nanoseconds();

  // set start time too
}

void EwarePanel::updateTopic() {

  std::cout << "update topic!" << std::endl;

  setTopic(output_topic_editor_->text());

  return;
}

void EwarePanel::setTopic(const QString &new_string) {

  // only take action if the name has changed.
  if (new_string == output_topic_) {
    return;
  }

  output_topic_ = new_string;

  if (trajectory_setpoint_pub_ != NULL) {
    trajectory_setpoint_pub_.reset();
    vis_path_pub_.reset();
  }

  // reset

  // if topic is empty, dont publish anything
  if (output_topic_ != "") {

    // create publishers
    trajectory_setpoint_pub_ =
        node_->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            output_topic_.toStdString() + "/fmu/in/trajectory_setpoint", 1);

    vis_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        output_topic_.toStdString() + "/viz/trajectory", 1);

    // create subscribers
    // rmw_qos_profile_t ....
  }

  // emit config changed signal
  Q_EMIT configChanged();
}

void EwarePanel::visualize_trajectory() {

  // create temp lissa
  double amplitude[4] = {amplitude_x->value(), amplitude_y->value(),
                         amplitude_z->value(),
                         amplitude_yaw->value() * M_PI / 180.0f};

  double omega[4] = {2 * M_PI * freq_x->value(), 2 * M_PI * freq_y->value(),
                     2 * M_PI * freq_z->value(), 2 * M_PI * freq_yaw->value()};

  double phi[4] = {
      M_PI / 180.0f * phi_x->value(), M_PI / 180.0f * phi_y->value(),
      M_PI / 180.0f * phi_z->value(), M_PI / 180.0f * phi_yaw->value()};

  double offset[4] = {offset_x->value(), offset_y->value(), offset_z->value(),
                      M_PI / 180.0f * offset_yaw->value()};

  Lissajous<double, 4> temp_lissa;

  temp_lissa.set_params(amplitude, omega, phi, offset);

  // start constructing the message
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "vicon/world";

  // evaluate the trajectory and publish it
  double T_max = 60.0;

  size_t N = 120;

  for (size_t i = 0; i < N; i++) {

    double t = (double(i) / double(N)) * T_max;

    double res[4];
    // evaluate returns the 
    temp_lissa.evaluate(res, t);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "vicon/world";
    pose.pose.position.x = res[1];
    pose.pose.position.y = res[0];
    pose.pose.position.z = -res[2];

    tf2::Quaternion q;
    q.setRPY(0, 0, 0.5 * M_PI - res[3]);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    path_msg.poses.push_back(pose);
  }

  // publish
  vis_path_pub_->publish(path_msg);
}

void EwarePanel::viz_timer_callback() {

  if (!(rclcpp::ok() && vis_path_pub_ != NULL)) {
    return;
  }

  visualize_trajectory();
}

void EwarePanel::timer_callback() {

  rclcpp::spin_some(node_);

  if (!(rclcpp::ok() && trajectory_setpoint_pub_ != NULL)) {
    return;
  }

  if (mode == Mode::STOPPED) {
    return;
  }

  // Go to the start trajectory
  if (mode == Mode::TAKEOFF) {
    // construct the setpoint
    msg.raw_mode = false;

    double res[4];
    lissa.evaluate(res, 0.0);

    for (size_t i = 0; i < 3; i++) {

      msg.position[i] = res[i];
      msg.velocity[i] = 0.0;
      msg.acceleration[i] = 0.0;
      msg.jerk[i] = 0.0;
    }

    msg.yaw = res[3];
    msg.yawspeed = 0.0;
    trajectory_setpoint_pub_->publish(msg);
    return;
  }

  if (mode == Mode::STARTED) {
    // construct the setpoint
    msg.raw_mode = false;

    auto now = node_->get_clock()->now().nanoseconds();

    double elapsed_s = 1e-9 * static_cast<double>(now - traj_start_time);

    double res_p[4];
    double res_v[4];
    double res_a[4];
    double res_j[4];
    lissa.evaluate(res_p, elapsed_s);
    lissa.evaluate(res_v, elapsed_s, 1);
    lissa.evaluate(res_a, elapsed_s, 2);
    lissa.evaluate(res_j, elapsed_s, 3);

    for (size_t i = 0; i < 3; i++) {

      msg.position[i] = res_p[i];
      msg.velocity[i] = res_v[i];
      msg.acceleration[i] = res_a[i];
      msg.jerk[i] = res_j[i];
    }

    msg.yaw = res_p[3];
    msg.yawspeed = res_v[3];
    trajectory_setpoint_pub_->publish(msg);
    return;
  }
  // Hover node
  if (mode == Mode::HOVER) {

    // publish the last position
    for (size_t i = 0; i < 3; i++) {

      msg.velocity[i] = 0.0;
      msg.acceleration[i] = 0.0;
      msg.jerk[i] = 0.0;
    }

    msg.yawspeed = 0.0;
    trajectory_setpoint_pub_->publish(msg);
    return;
  }
}

void EwarePanel::save(rviz_common::Config config) const {

  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", output_topic_);
}


void EwarePanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);

  QString topic;
  if (config.mapGetString("Topic", &topic)) {

    output_topic_editor_->setText(topic);
    updateTopic();
  }
}

} // namespace dasc_robot_gui

PLUGINLIB_EXPORT_CLASS(dasc_robot_gui::EwarePanel, rviz_common::Panel)