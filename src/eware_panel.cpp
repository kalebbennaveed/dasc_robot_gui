#include "eware_panel.hpp"

#include "pluginlib/class_list_macros.hpp"

// Create two publishers
// 1. Eware mission status to publish the EwareMissionStatus.msg to signal onboard computer to start the mission
// 2. (Might not need) Trajectory Visualizer


namespace dasc_robot_gui {

using std::placeholders::_1;

EwarePanel::EwarePanel(QWidget *parent) : rviz_common::Panel(parent) {

  // construct the layout

  // topic layout
  QHBoxLayout *topic_layout = new QHBoxLayout;
  topic_layout->addWidget(new QLabel("Robot Namespace:"));
  output_topic_editor_ = new QLineEdit;
  topic_layout->addWidget(output_topic_editor_);

  // buttons
  QHBoxLayout *button_layout = new QHBoxLayout;
  start = new QPushButton("Start Mission", this);
  stop = new QPushButton("Stop Mission", this);
  stay = new QPushButton("Stay at Ground", this);

  for (auto b : {start, stop, stay}) {
    button_layout->addWidget(b);
  }

  // stack it all together
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  layout->addLayout(button_layout);
  setLayout(layout);

  // create the timer
  QTimer *output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(timer_callback()));

  // Connect the Push button to the specific modes
  connect(start, &QPushButton::clicked, this, [this]() { mode = Mode::STARTED;});
  connect(stop, &QPushButton::clicked, this, [this]() { mode = Mode::STOPPED;});
  connect(stay, &QPushButton::clicked, this, [this]() { mode = Mode::GROUNDED;});

  // connect the slots
  connect(output_topic_editor_, SIGNAL(editingFinished()), this,
          SLOT(updateTopic()));

  // create the node
  node_ = std::make_shared<rclcpp::Node>("eware_panel_node");

  // start the main timer
  output_timer->start(10); // ms
  // viz_timer->start(10);   // ms
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

  if (eware_mission_status_pub_ != NULL) {
    eware_mission_status_pub_.reset();
  }

  // if topic is empty, dont publish anything
  if (output_topic_ != "") {

    eware_mission_status_pub_ = node_->create_publisher<dasc_msgs::msg::EwareMissionStatus>(
      output_topic_.toStdString() + "/gs/eware_mission_status", 1);
    // create subscribers
    // rmw_qos_profile_t ....
  }

  // emit config changed signal
  Q_EMIT configChanged();
}


void EwarePanel::timer_callback() {

  // Unlike rclcpp::spin, rclcpp::spin_some processess only the work available at the moment and then returns immediately
  rclcpp::spin_some(node_);

  // If the trajectory_setpoint_pub_ is NULL, it means that publisher has not been properly initialized 
  if (!(rclcpp::ok() && eware_mission_status_pub_ != NULL)) {
    return;
  }

  // if pressed grounded (land immediately)
  if (mode == Mode::GROUNDED) {
      eware_mission_status_msg.mission_start = false;
      eware_mission_status_msg.mission_quit = false;
      eware_mission_status_pub_->publish(eware_mission_status_msg);  
    return;
  }

  // Start the mission
  if (mode == Mode::STARTED) {
    eware_mission_status_msg.mission_start = true;
    eware_mission_status_pub_->publish(eware_mission_status_msg);
    return;
  }

  // Stop the mission and land immediately
  if (mode == Mode::STOPPED) {
    eware_mission_status_msg.mission_quit = true;
    eware_mission_status_pub_->publish(eware_mission_status_msg);
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