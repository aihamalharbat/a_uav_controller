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
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
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

    state_sub = nh.subscribe<mavros_msgs::State>                                // Read Statues
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
            set_mode_client.exists() &&
            set_mode_client.isValid()){
            set_mode_client.call(offb_set_mode);
            ROS_INFO_ONCE("Offboard enabled");
        }
        if (!current_state_.armed &&
            arming_client.exists() &&
            arming_client.isValid()){
            arming_client.call(arm_cmd);
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
        Eigen::Vector4d ActCmdsNormalized = normalizeActCmds(&ActCmds);
        mavros_msgs::ActuatorControlPtr actuator_msg(new mavros_msgs::ActuatorControl);
        actuator_msg->group_mix = 0;
        actuator_msg->controls[0] = ActCmdsNormalized[0];
        actuator_msg->controls[1] = ActCmdsNormalized[1];
        actuator_msg->controls[2] = ActCmdsNormalized[2];
        actuator_msg->controls[3] = ActCmdsNormalized[3];
        actuator_msg->header.stamp = odometry_msg->header.stamp;
    // Debug message
    ROS_INFO("Pre-Published! (from V2)");                           // Working till here
    //  Publish Actuators message
    ActCmds_pub_.publish(actuator_msg);                             // The publisher is the problem
    ROS_INFO("Published! (from V2)");
    }

}

Eigen::Vector4d controller_node::normalizeActCmds(Eigen::VectorXd *wrench) {
    //  Assuming that:
    //  wrench = [roll_torque; pitch_torque; yaw_torque; thrust]
    Eigen::Vector4d normalizedWrench;
    //  These values are form this matlab code:
//    %   From iris.sdf
//    x_position = 0.13;
//    y_position = 0.22;
//    K_motor = 5.84e-06;
//    K_moment = 0.06;
//    max_vel = 1100;     % rad/s
//    max_force = K_motor * (max_vel^2)
//    max_x_torque = 2 * max_force * x_position
//    max_y_torque = 2 * max_force * y_position
//    max_thrust = 4 * max_force
//    max_yaw_torque = 4 * K_moment * max_force
    // TODO: move _max_ to static parameters
    double _max_roll_torque = 1.8373;
    double _max_pitch_torque = 3.1092;
    double _max_yaw_torque = 1;
    double _max_thrust = 28.2656;
    // Normalize
    normalizedWrench[0] = (*wrench)[0] / _max_roll_torque;      // normalize roll torque
    normalizedWrench[1] = (*wrench)[1] / _max_pitch_torque;     // normalize pitch torque
    normalizedWrench[2] = (*wrench)[2] / _max_yaw_torque;       // normalize yaw torque
    normalizedWrench[3] = (*wrench)[3] / _max_thrust;           // Normalize F_z aka thrust
    // Limit the torques to [-1,1]
    // TODO: use std:max and std::min instead of ifs and for
    for (int i = 0; i < 4; ++i) {
        if (normalizedWrench[i] > 1){
            normalizedWrench[i] = 1;
        }
        else if (normalizedWrench(i) < -1) {
            normalizedWrench[i] = -1;
        }
    }
    // Limit the thrust to [0,1]
    if (normalizedWrench[3] > 1){
        normalizedWrench[3] = 1;
    }
    else if (normalizedWrench[3] < 0) {
        normalizedWrench[3] = 0;
    }
    return normalizedWrench;
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