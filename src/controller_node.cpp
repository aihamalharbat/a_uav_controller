//
// Created by aalharbat on 14-10-20.
//

#include "controller_node.h"
#include "a_uav_controller/controller.h"
#include <mav_msgs/Actuators.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include "mavros_msgs/ActuatorControl.h"
#include <ros/ros.h>


controller_node::controller_node() {
    ros::NodeHandle nh;

    // Subscriptions:
    cmd_pose_sub_ =nh_.subscribe(
            mav_msgs::default_topics::COMMAND_POSE, 1,
            &controller_node::CommandPoseCallback, this);

    cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
            mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
            &controller_node::MultiDofJointTrajectoryCallback, this);

    odometry_sub_ =nh_.subscribe(
            "/mavros/odometry/in", 1,
            &controller_node::OdometryCallbackV2, this);

    // Publications:
    motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(         // Not used
            mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

    ActCmds_pub_ = nh_.advertise<mavros_msgs::ActuatorControl>
            ("mavros/actuator_control", 10);

    command_timer_ = nh_.createTimer(
            ros::Duration(0), &controller_node::TimedCommandCallback, this,
            true, false);
}

controller_node::~controller_node() = default;

void controller_node::CommandPoseCallback(
        const geometry_msgs::PoseStampedConstPtr& pose_msg) {                   // When a command is received
    // Clear all pending commands.
    command_timer_.stop();
    commands_.clear();
    command_waiting_times_.clear();

    mav_msgs::EigenTrajectoryPoint eigen_reference;
    mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
    commands_.push_front(eigen_reference);

    controller_.setTrajectoryPoint(commands_.front());          // Send the command to controller_ obj
    commands_.pop_front();
}

void controller_node::MultiDofJointTrajectoryCallback(
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

void controller_node::OdometryCallback(
        const nav_msgs::OdometryConstPtr& odometry_msg) {
    ROS_INFO_ONCE("Controller got first odometry message.");

    mav_msgs::EigenOdometry odometry;
    mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry);
    controller_.setOdometry(odometry);

    Eigen::VectorXd ref_rotor_velocities;
    controller_.calculateRotorVelocities(&ref_rotor_velocities);

    // Todo(ffurrer): Do this in the conversions header.
    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

    actuator_msg->angular_velocities.clear();
    for (int i = 0; i < ref_rotor_velocities.size(); i++) {
        actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
    }

    actuator_msg->header.stamp = odometry_msg->header.stamp;

    motor_velocity_reference_pub_.publish(actuator_msg);
    ROS_INFO("Published!");
}

void controller_node::OdometryCallbackV2(
        const nav_msgs::OdometryConstPtr& odometry_msg) {
    ROS_INFO_ONCE("Controller got first odometry message.");

    mav_msgs::EigenOdometry odometry;
    mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry);
    controller_.setOdometry(odometry);

    Eigen::VectorXd ActCmds;
    controller_.calculateActCmds(&ActCmds);

    // Todo(ffurrer): Do this in the conversions header.
//    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
    mavros_msgs::ActuatorControlPtr actuator_msg(new mavros_msgs::ActuatorControl);
    actuator_msg->group_mix = 0;
    actuator_msg->controls[0] = ActCmds[0];
    actuator_msg->controls[1] = ActCmds[1];
    actuator_msg->controls[2] = ActCmds[2];
    actuator_msg->controls[3] = ActCmds[3];
//    ROS_INFO("Again, Thrust = %f", actuator_msg->controls[0]);
    actuator_msg->header.stamp = odometry_msg->header.stamp;
    ROS_INFO("Pre-Published! (from V2)");                           // Working till here

    ActCmds_pub_.publish(actuator_msg);                             // The publisher is the problem
    ROS_INFO("Published! (from V2)");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node");

    controller_node
            controller_node_;

    ros::spin();

    return 0;
}