//
// Created by aalharbat on 14-10-20.
//

#include "a_uav_controller/controller_node.h"
#include "a_uav_controller/controller.h"
#include <mav_msgs/Actuators.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include "mavros_msgs/ActuatorControl.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include <ros/ros.h>


controller_node::controller_node() {
    ros::NodeHandle nh;
    // Services and connection
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    // Subscriptions:
    cmd_pose_sub_ =nh_.subscribe(                                               // Read command
            mav_msgs::default_topics::COMMAND_POSE, 1,
            &controller_node::CommandPoseCallback, this);

    cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(                        // Not used
            mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
            &controller_node::MultiDofJointTrajectoryCallback, this);

    odometry_sub_ =nh_.subscribe(                                               // Read odometry
            "/mavros/odometry/in", 1,
            &controller_node::OdometryCallbackV2, this);

    state_sub_ = nh.subscribe<mavros_msgs::State>                                // Read Statues
            ("mavros/state", 10,
             &controller_node::stateCallBack, this);

    // Publications:
    motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(         // Not used
            mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    ActCmds_pub_ = nh_.advertise<mavros_msgs::ActuatorControl>
            ("mavros/actuator_control", 10);

    command_timer_ = nh_.createTimer(
            ros::Duration(0), &controller_node::TimedCommandCallback, this,
            true, false);
    // Initialize Dynamic Reconfigure
    gainsServer.setCallback(boost::bind(&controller_node::dynamicReconfigureCallback, this, _1, _2));
    connected_ = false;
    secureConnection();
}

controller_node::~controller_node() = default;                                  // Deconstruct

void controller_node::secureConnection() {
    ros::Rate rate(20);
    // Wait for FCU connection
    while(ros::ok() && !current_state_.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected!");

    // Fill the buffer
    mavros_msgs::ActuatorControl tempMsg;
    tempMsg.group_mix = 0;
    tempMsg.controls[1] = 0;
    tempMsg.controls[2] = 0;
    tempMsg.controls[3] = 0;
    tempMsg.controls[4] = 0;
    for(int i = 100; ros::ok() && i > 0; --i){
        ActCmds_pub_.publish(tempMsg);
        ros::spinOnce();                        //resposible to handle communication events, e.g. arriving messages
        rate.sleep();
    }
    ROS_INFO_ONCE("Done filling buffer!");

    /*  *********************************
     *              ARM & OFFOARD
     *  *********************************
     */
    ros::Time last_request = ros::Time::now();
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ROS_INFO_ONCE("Start ARMING and OB.");
//    while (!current_state_.armed && current_state_.mode != "OFFBOARD"){
//        if(set_mode_client.call(offb_set_mode) &&
//            (offb_set_mode.response.mode_sent) &&
//            (ros::Time::now() - last_request > ros::Duration(5.0)) ){
//            ROS_INFO("Offboard enabled");
//            last_request = ros::Time::now();
//        }
//        else if (!current_state_.armed &&
//            (ros::Time::now() - last_request > ros::Duration(5.0)) &&
//            (arming_client.call(arm_cmd) &&
//                    arm_cmd.response.success)) {
//                    ROS_INFO("Vehicle armed");
//                    last_request = ros::Time::now();
//
//        }
//        ros::spinOnce();
//        rate.sleep();
//        ROS_INFO_ONCE("In the loop for arming");
//    }
    while (current_state_.mode != "OFFBOARD" && !current_state_.armed ){
        if (current_state_.mode != "OFFBOARD" &&
            set_mode_client_.exists() &&
            set_mode_client_.isValid()){
            set_mode_client_.call(offb_set_mode);
            ROS_INFO_ONCE("Offboard enabled");
        }
        if (!current_state_.armed &&
            arming_client_.exists() &&
            arming_client_.isValid()){
            arming_client_.call(arm_cmd);
            ROS_INFO_ONCE("Vehicle armed");
        }
        ros::spinOnce();
    }
    ROS_INFO("Out of arming and OFFb loop");
    connected_ = true;
}


void controller_node::CommandPoseCallback(
        const geometry_msgs::PoseStampedConstPtr& pose_msg) {                   // When a command is received
    // Clear all pending commands.
    command_timer_.stop();
    commands_.clear();
    command_waiting_times_.clear();

    mav_msgs::EigenTrajectoryPoint eigen_reference;
    mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
    commands_.push_front(eigen_reference);

    ROS_INFO_ONCE("Controller got first command message.");
    controller_.setTrajectoryPoint(commands_.front());          // Send the command to controller_ obj
    commands_.pop_front();
}

void controller_node::MultiDofJointTrajectoryCallback(                           // Not used
        const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
    // Clear all pending commands.
    command_timer_.stop();
    commands_.clear();
    command_waiting_times_.clear();

    const size_t n_commands = msg->points.size();

    if (n_commands < 1) {
        ROS_WARN_STREAM(
                "Got MultiDOFJointTrajectory message, but message has no points.");
        return;
    }

    mav_msgs::EigenTrajectoryPoint eigen_reference;
    mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
    commands_.push_front(eigen_reference);

    for (size_t i = 1; i < n_commands; ++i) {
        const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before =
                msg->points[i - 1];
        const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference =
                msg->points[i];

        mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

        commands_.push_back(eigen_reference);
        command_waiting_times_.push_back(current_reference.time_from_start -
                                         reference_before.time_from_start);
    }

    // We can trigger the first command immediately.
    controller_.setTrajectoryPoint(commands_.front());
    commands_.pop_front();

    if (n_commands > 1) {
        command_timer_.setPeriod(command_waiting_times_.front());
        command_waiting_times_.pop_front();
        command_timer_.start();
    }
}

void controller_node::TimedCommandCallback(const ros::TimerEvent& e) {
    if (commands_.empty()) {
        ROS_WARN("Commands empty, this should not happen here");
        return;
    }

    controller_.setTrajectoryPoint(commands_.front());
    commands_.pop_front();
    command_timer_.stop();
    if (!command_waiting_times_.empty()) {
        command_timer_.setPeriod(command_waiting_times_.front());
        command_waiting_times_.pop_front();
        command_timer_.start();
    }
}

void controller_node::OdometryCallback(                                     // read odometry and take action
        const nav_msgs::OdometryConstPtr& odometry_msg) {                   // THE MAIN connection to controller class

    //  Debug message
    ROS_INFO_ONCE("Controller got first odometry message.");
    // send odometry to controller_ obj
    mav_msgs::EigenOdometry odometry;
    mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry);
    controller_.setOdometry(odometry);
    //  calculate controller output
    Eigen::VectorXd ref_rotor_velocities;
    controller_.calculateRotorVelocities(&ref_rotor_velocities);

    /* Todo(ffurrer): Do this in the conversions header. */
    //  prepare actuators message
    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
    actuator_msg->angular_velocities.clear();
    for (int i = 0; i < ref_rotor_velocities.size(); i++) {
        actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
    }
    actuator_msg->header.stamp = odometry_msg->header.stamp;
    //  Publish Actuators message
    motor_velocity_reference_pub_.publish(actuator_msg);
    ROS_INFO("Published!");
}

void controller_node::OdometryCallbackV2(                                       // read odometry and take action
        const nav_msgs::OdometryConstPtr& odometry_msg) {                       // THE MAIN connection to controller class
    if (connected_) {
        //  Debug message
        ROS_INFO_ONCE("Controller got first odometry message.");
        // send odometry to controller_ obj
        mav_msgs::EigenOdometry odometry;
        mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry);
        controller_.setOdometry(odometry);
        //  calculate controller output
        Eigen::VectorXd ActCmds;
        controller_.calculateActCmds(&ActCmds);
        // Todo(ffurrer): Do this in the conversions header.
        //  prepare actuators message
        mavros_msgs::ActuatorControlPtr actuator_msg(new mavros_msgs::ActuatorControl);
        actuator_msg->group_mix = 0;
        actuator_msg->controls[0] = ActCmds[0];
        actuator_msg->controls[1] = - ActCmds[1];
        actuator_msg->controls[2] = 0;//ActCmds[2];
        actuator_msg->controls[3] = ActCmds[3];
        actuator_msg->header.stamp = odometry_msg->header.stamp;
        // Debug message
        ROS_INFO("Tau_x = %f", ActCmds[0]);
        ROS_INFO("Tau_y = %f", ActCmds[1]);
        ROS_INFO("Tau_z = %f", ActCmds[2]);
        ROS_INFO("Thrust = %f", ActCmds[3]);
        //  Publish Actuators message
        ActCmds_pub_.publish(actuator_msg);

//        // Testing this fixed act_msg
//        mavros_msgs::ActuatorControl act_cmd;
//        act_cmd.group_mix = 0;
//        act_cmd.controls[0] = 0;
//        act_cmd.controls[1] = 0;
//        act_cmd.controls[2] = 0;
//        act_cmd.controls[3] = 0;
//        ActCmds_pub_.publish(act_cmd);
//        ROS_INFO("Published! (from V2)");
    }
}

void controller_node::stateCallBack(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
    if (msg->armed){
        ROS_INFO_ONCE("ARMED - State_msg.");
    }
    else {
        ROS_INFO("NOT ARMED - State_msg.");
    }

    if (msg->mode == "OFFBOARD"){
        ROS_INFO_ONCE("OFFBOARD - State_msg.");
    }
    else {
        ROS_INFO("NOT OFFBOARD - State_msg.");
    }
}

void controller_node::dynamicReconfigureCallback(const a_uav_controller::parametersConfig &config,
                                                               const uint32_t level) {
    controller_.setKPositionGain(Eigen::Vector3d(config.K_p_x, config.K_p_y, config.K_p_z));
    controller_.setKVelocityGain(Eigen::Vector3d(config.K_v_x, config.K_v_y, config.K_v_z));
    controller_.setKAttitudeGain(Eigen::Vector3d(config.K_R_x, config.K_R_y, config.K_R_z));
    controller_.setKAngularRateGain(Eigen::Vector3d(config.K_w_x, config.K_w_y, config.K_w_z));
    setThrust(config.Thrust);
    setXToruqe(config.xTorque);
    setYToruqe(config.yTorque);
    ROS_INFO("Gains changed!");
}

void controller_node::setThrust(double thrust) {
    controller_node::thrust = thrust;
}

void controller_node::setXToruqe(double xToruqe) {
    controller_node::xToruqe = xToruqe;
}

void controller_node::setYToruqe(double yToruqe) {
    controller_node::yToruqe = yToruqe;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node");

    controller_node
            controller_node_;
    ros::Rate rate(20);
    while (ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}