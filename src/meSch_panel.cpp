#include "meSch_panel.hpp"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QRadioButton>
#include <QTimer>
#include <QVBoxLayout>
#include <stdio.h>

#include <memory>

#include "pluginlib/class_list_macros.hpp"

namespace dasc_robot_gui {

using std::placeholders::_1;

meSchPanel::meSchPanel(QWidget *parent) : rviz_common::Panel(parent) {

    // Robot 1 name
    QHBoxLayout *robot_1_layout = new QHBoxLayout;
    robot_1_layout->addWidget(new QLabel("Robot 1 Name:"));
    robot_1_editor_ = new QLineEdit;
    robot_1_layout->addWidget(robot_1_editor_);  


//   QHBoxLayout *topic_layout = new QHBoxLayout;
//   topic_layout->addWidget(new QLabel("Robot Namespace:"));
//   output_topic_editor_ = new QLineEdit;
//   topic_layout->addWidget(output_topic_editor_);

    // Robot 2 name
    QHBoxLayout *robot_2_layout = new QHBoxLayout;
    robot_2_layout->addWidget(new QLabel("Robot 2 Name:"));
    robot_2_editor_ = new QLineEdit;
    robot_2_layout->addWidget(robot_2_editor_);    

    // Robot 3 name
    QHBoxLayout *robot_3_layout = new QHBoxLayout;
    robot_3_layout->addWidget(new QLabel("Robot 3 Name:"));
    robot_3_editor_ = new QLineEdit;
    robot_3_layout->addWidget(robot_3_editor_);    

    // To list the number of robots
    QHBoxLayout *robot_num_layout = new QHBoxLayout;
    robot_num_layout->addWidget(new QLabel("Number of Robots:"));
    robot_num_editor_ = new QLineEdit;
    robot_num_layout->addWidget(robot_num_editor_);

    // Initilialize button layout
    QHBoxLayout *init_topic_layout = new QHBoxLayout;
    init_topic_button_ = new QPushButton("Initialize Topics", this);
    init_topic_button_->setDisabled(false);
    init_topic_layout->addWidget(init_topic_button_);

    // state labels
    QHBoxLayout *status_layout = new QHBoxLayout;
    status_label_ = new QLabel("status: Fill name -> num (at least 2) -> init");
    status_layout->addWidget(status_label_);

    // create the mission preview buttons
    QHBoxLayout *sim_layout = new QHBoxLayout;
    start_sim_button_ = new QPushButton("Mission Preview", this);
    stop_sim_button_ = new QPushButton("Stop Preview", this);
    start_sim_button_->setDisabled(false);
    stop_sim_button_->setDisabled(false);
    sim_layout->addWidget(start_sim_button_);
    sim_layout->addWidget(stop_sim_button_);


    // Create the mission buttons
    QHBoxLayout *mission_layout = new QHBoxLayout;
    start_mis_button_ = new QPushButton("Start Mission", this);
    stop_mis_button_ = new QPushButton("Stop Mission / Land all", this);
    start_mis_button_->setDisabled(true);
    stop_mis_button_->setDisabled(false);
    sim_layout->addWidget(start_mis_button_);
    sim_layout->addWidget(stop_mis_button_);

    // create the disarm button
    QHBoxLayout *disarm_layout = new QHBoxLayout;
    disarm_button_ = new QPushButton("&Disarm all", this);
    disarm_button_->setDisabled(false);
    disarm_layout->addWidget(disarm_button_);

    // Lay out the topic field above the control widget.
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addLayout(robot_1_layout);
    layout->addLayout(robot_2_layout);
    layout->addLayout(robot_3_layout);
    layout->addLayout(robot_num_layout);
    layout->addLayout(init_topic_layout);
    layout->addLayout(status_layout);
    layout->addLayout(sim_layout);
    layout->addLayout(mission_layout);
    layout->addLayout(disarm_layout);
    setLayout(layout);

    // Create a timer for sending the output.  
    QTimer *output_timer = new QTimer(this);

    // Next we make signal/slot connections.
    connect(robot_1_editor_, SIGNAL(editingFinished()), this,
            SLOT(UpdateRobot1Name()));    

    connect(robot_2_editor_, SIGNAL(editingFinished()), this,
            SLOT(UpdateRobot2Name())); 

    connect(robot_3_editor_, SIGNAL(editingFinished()), this,
            SLOT(UpdateRobot3Name())); 

    connect(robot_num_editor_, SIGNAL(editingFinished()), this,
            SLOT(UpdateRobotNumber()));


    connect(init_topic_button_, &QPushButton::clicked, this, [this]() {
        UpdateTopic()
        status_label_->setText("state: Topics Updated");
        mode = Mode::TOPICS_INIT;
    });

    connect(start_sim_button_, &QPushButton::clicked, this, [this]() {
        mode = Mode::SIMSTART;
        start_mis_button_->setDisabled(true);
    });

    connect(stop_sim_button_, &QPushButton::clicked, this, [this]() {
        this->all_commander_set_state(px4_msgs::msg::CommanderSetState::STATE_DISARMED);
        mode = Mode::STOPPED;
        start_mis_button_->setDisabled(false);
    });

    connect(start_mis_button_, &QPushButton::clicked, this, [this]() {
        mode = Mode::STARTED;
        start_mis_button_->setDisabled(true); // Disable the mission start button
        start_sim_button_->setDisabled(true); //
        stop_sim_button_->setDisabled(true);
    });


    connect(stop_mis_button_, &QPushButton::clicked, this, [this]() {
        this->all_commander_set_state(px4_msgs::msg::CommanderSetState::STATE_LAND);
        mode = Mode::STOPPED;
    });

    connect(disarm_button_, &QPushButton::clicked, this, [this]() {
        this->all_commander_set_state(px4_msgs::msg::CommanderSetState::STATE_DISARMED);
        mode = Mode::GROUNDED;
        status_label_ = new QLabel("status: Fill name -> num (at least 2) -> init");
        start_mis_button_->setDisabled(true); // Disable the mission start button
        stop_mis_button_->setDisabled(false);
        start_sim_button_->setDisabled(false); //
        stop_sim_button_->setDisabled(false);
        disarm_button_->setDisabled(false); 

    });

    connect(output_timer, SIGNAL(timeout()), this, SLOT(timer_callback()));
    // Start the main timer.
    output_timer->start(100); // ms

    // Create the node
    node_ = std::make_shared<rclcpp::Node>("meSch_gui_node");
}

void meSchPanel::timer_callback() {

    rclcpp::spin_some(node_);

    // check if the connection is still alive
    rclcpp::Time now_ = node_->get_clock()->now();
    const uint64_t status_timeout_ns = 1e9;
    if (now_.nanoseconds() - last_timestamp_commander_status_ >
        status_timeout_ns) {

        // set status to comms lost
        start_mis_button_->setDisabled(true);
        status_label_->setText("state: COMMS_LOST");
    }

    if (!(rclcpp::ok() && meSch_mission_status_pub_ != NULL)) {
        return;
    }

    // If Topics initialized -> enable the start mission button
    if (mode == Mode::TOPICS_INIT){
        if (robot_num_int_ == 2){
            if (robot_1_commander_status_ == px4_msgs::msg::CommanderStatus::STATE_OFFBOARD
                && robot_2_commander_status_ == px4_msgs::msg::CommanderStatus::STATE_OFFBOARD){
                // change the mode to offboard
                mode == Mode::OFFBOARD;

                // Enable the mission button
                start_mis_button_->setDisabled(false)
            }
        }else if (robot_num_int_ == 3){
            if (robot_1_commander_status_ == px4_msgs::msg::CommanderStatus::STATE_OFFBOARD
                && robot_2_commander_status_ == px4_msgs::msg::CommanderStatus::STATE_OFFBOARD
                && robot_3_commander_status_ == px4_msgs::msg::CommanderStatus::STATE_OFFBOARD){
                // change the mode to offboard
                mode == Mode::OFFBOARD;

                // Enable the mission button
                start_mis_button_->setDisabled(false);
            }        
        }
    }

    if (mode == Mode::OFFBOARD){
        if (robot_num_int_ == 2){
            if (robot_1_commander_status_ != px4_msgs::msg::CommanderStatus::STATE_OFFBOARD
                || robot_2_commander_status_ != px4_msgs::msg::CommanderStatus::STATE_OFFBOARD){

                // land all
                this->all_commander_set_state(px4_msgs::msg::CommanderSetState::STATE_LAND);
                mode = Mode::STOPPED;
                start_mis_button_->setDisabled(true);
            }
        }else if (robot_num_int_ == 3){
            if (robot_1_commander_status_ != px4_msgs::msg::CommanderStatus::STATE_OFFBOARD
                || robot_2_commander_status_ != px4_msgs::msg::CommanderStatus::STATE_OFFBOARD
                || robot_3_commander_status_ != px4_msgs::msg::CommanderStatus::STATE_OFFBOARD){

                // land all
                this->all_commander_set_state(px4_msgs::msg::CommanderSetState::STATE_LAND);
                mode = Mode::STOPPED;
                start_mis_button_->setDisabled(true);
            }        
        }
    }



  // if pressed INIT OR just hovering at starting point then
  if (mode == Mode::TOPICS_INIT || mode == Mode::OFFBOARD) {
      mesch_mission_status_msg.mission_start = false;
      mesch_mission_status_msg.mission_quit = false;
      mesch_mission_status_pub->publish(mesch_mission_status_msg);  
    return;
  }

  // Start the mission
  if (mode == Mode::STARTED || mode == Mode::SIMSTART) {
    mesch_mission_status_msg.mission_start = true;
    mesch_mission_status_msg.mission_quit = false;
    mesch_mission_status_pub->publish(mesch_mission_status_msg);
    return;
  }

  // Stop the mission and land immediately
  if (mode == Mode::STOPPED) {
    mesch_mission_status_msg.mission_start = false;
    mesch_mission_status_msg.mission_quit = true;
    mesch_mission_status_pub->publish(mesch_mission_status_msg);
    return;
  }

}

void meSchPanel::UpdateRobot1Name(){
    robot_1_name_ = robot_1_editor_->text();
}

void meSchPanel::UpdateRobot2Name(){
    robot_2_name_ = robot_2_editor_->text();
}

void meSchPanel::UpdateRobot3Name(){
    robot_3_name_ = robot_3_editor_->text();
}

void meSchPanel::UpdateRobotNumber(){
    robot_num_ = robot_num_editor_->text();
    
    // Store as an int
    robot_num_int_ = robot_num_.toInt());

    // Clear the old names
    robot_names_.clear()

    if (robot_num_int_ == 2){
        robot_names_.append(robot_1_name_);
        robot_names_.append(robot_2_name_);
    } else if (robot_num_int_ == 3) {
        robot_names_.append(robot_1_name_);
        robot_names_.append(robot_2_name_);
        robot_names_.append(robot_3_name_);       
    }
}

void meSchPanel::UpdateTopic(){
 
    // Reset Topics if needed
    ResetTopics();

    // Create Publishers and Subscribers
    if (!robot_names_.empty()){
        
        // For Publihers 

        // Mission status pub
        meSch_mission_status_pub_ = node_->create_publisher<dasc_msgs::msg::MeschMissionStatus>(
            output_topic_.toStdString() + "/gs/mesch_mission_status", 1);

        // Clear the vector
        commander_set_state_pub_vec_.clear();

        for (const QString &topic_name_ : robot_names_){

            commander_set_state_pub_ =
                node_->create_publisher<px4_msgs::msg::CommanderSetState>(
                    topic_name_.toStdString() + "/fmu/in/commander_set_state", 1);

            commander_set_state_pub_vec_.append(commander_set_state_pub_);
        }

        // For Subscribers
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),
                                qos_profile);
        
        if (robot_num_int_ == 2){
            
            // First robot sub
            auto new_topic = robot_names_[0].toStdString() + "/fmu/out/commander_status";
            std::cout << "Subscribing to: " << new_topic << std::endl;
            commander_status_sub_ =
                node_->create_subscription<px4_msgs::msg::CommanderStatus>(
                    new_topic, qos,
                    std::bind(&meSchPanel::robot_1_commander_status_cb, this, _1));

            // Second robot sub
            new_topic = robot_names_[1].toStdString() + "/fmu/out/commander_status";
            std::cout << "Subscribing to: " << new_topic << std::endl;
            commander_status_sub_ =
                node_->create_subscription<px4_msgs::msg::CommanderStatus>(
                    new_topic, qos,
                    std::bind(&meSchPanel::robot_2_commander_status_cb, this, _1));

        } else if (robot_num_int_ == 3){
                // First robot sub
                auto new_topic = robot_names_[0].toStdString() + "/fmu/out/commander_status";
                std::cout << "Subscribing to: " << new_topic << std::endl;
                commander_status_sub_ =
                    node_->create_subscription<px4_msgs::msg::CommanderStatus>(
                        new_topic, qos,
                        std::bind(&meSchPanel::robot_1_commander_status_cb, this, _1));

                // Second robot sub
                new_topic = robot_names_[1].toStdString() + "/fmu/out/commander_status";
                std::cout << "Subscribing to: " << new_topic << std::endl;
                commander_status_sub_ =
                    node_->create_subscription<px4_msgs::msg::CommanderStatus>(
                        new_topic, qos,
                        std::bind(&meSchPanel::robot_2_commander_status_cb, this, _1));
                
                // Second robot sub
                new_topic = robot_names_[2].toStdString() + "/fmu/out/commander_status";
                std::cout << "Subscribing to: " << new_topic << std::endl;
                commander_status_sub_ =
                    node_->create_subscription<px4_msgs::msg::CommanderStatus>(
                        new_topic, qos,
                        std::bind(&meSchPanel::robot_3_commander_status_cb, this, _1));                
        }
    }

    // rviz_common::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz_common::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();

}


void meSchPanel::ResetTopics(){

    // Reset the topics if they are already not null

    // Mission status pub
    if (meSch_mission_status_pub_ != NULL) {
    meSch_mission_status_pub_.reset();
    }    

    // Commander set state pub for each robot
    if (!commander_set_state_pub_vec_.empty()){
        for (const auto &commander_set_state_pub_ : commander_set_state_pub_vec_){
            commander_set_state_pub_.reset();
        }
    }

}

void meSchPanel::robot_1_commander_status_cb(
    const px4_msgs::msg::CommanderStatus::SharedPtr msg) {
        
    last_timestamp_commander_status_ = node_->get_clock()->now().nanoseconds();

    // Store the robot 1 commander state 
    robot_1_commander_status_ = msg->state;
}

void meSchPanel::robot_2_commander_status_cb(
    const px4_msgs::msg::CommanderStatus::SharedPtr msg) {
        
    last_timestamp_commander_status_ = node_->get_clock()->now().nanoseconds();

    // Store the robot 1 commander state 
    robot_2_commander_status_ = msg->state;
}

void meSchPanel::robot_3_commander_status_cb(
    const px4_msgs::msg::CommanderStatus::SharedPtr msg) {
        
    last_timestamp_commander_status_ = node_->get_clock()->now().nanoseconds();

    // Store the robot 1 commander state 
    robot_3_commander_status_ = msg->state;
}


void meSchPanel::all_commander_set_state(uint8_t new_state) {
  if (rclcpp::ok() && mode == mode::OFFBOARD) {
    px4_msgs::msg::CommanderSetState msg;
    msg.new_state = new_state;
    for (const auto &commander_set_state_pub_ : commander_set_state_pub_vec_){
        commander_set_state_pub_->publish(msg);
    }
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void meSchPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Robot_1", robot_1_name_);
  config.mapSetValue("Robot_2", robot_2_name_);
  config.mapSetValue("Robot_3", robot_3_name_);
}

// Load all configuration data for this panel from the given Config object.
void meSchPanel::load(const rviz_common::Config &config) {
  rviz_common::Panel::load(config);
//   QString robot_1_name, robot_2_name, robot_3_name;
  if (config.mapGetString("Robot_1", &robot_1_name_)) {
    robot_1_editor_->setText(robot_1_name_);
  }
  if (config.mapGetString("Robot_2", &robot_2_name_)) {
    robot_2_editor_->setText(robot_2_name_);
  }  
  if (config.mapGetString("Robot_3", &robot_3_name_)) {
    robot_3_editor_->setText(robot_3_name_);
  } 
  UpdateRobotNumber();
}

} // end namespace dasc_robot_gui

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
PLUGINLIB_EXPORT_CLASS(dasc_robot_gui::meSchPanel, rviz_common::Panel)
// END_TUTORIAL
