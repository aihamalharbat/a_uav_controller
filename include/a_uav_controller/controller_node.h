//
// Created by aalharbat on 14-10-20.
//

#ifndef A_UAV_CONTROLLER_CONTROLLER_NODE_H
#define A_UAV_CONTROLLER_CONTROLLER_NODE_H

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include "a_uav_controller/controller.h"
#include "mavros_msgs/State.h"
#include <dynamic_reconfigure/server.h>
#include <a_uav_controller/parametersConfig.h>

class controller_node {
public:
    controller_node();
    virtual ~controller_node();

private:
    ros::NodeHandle nh_;
    controller controller_;
    std::string namespace_;
    // subscribers
    ros::Subscriber cmd_trajectory_sub_;
    ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
    ros::Subscriber cmd_pose_sub_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber state_sub_;
    // Publishers
    ros::Publisher motor_velocity_reference_pub_;
    ros::Publisher ActCmds_pub_;
    // Services
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    dynamic_reconfigure::Server<a_uav_controller::parametersConfig> gainsServer;

    mav_msgs::EigenTrajectoryPointDeque commands_;
    std::deque<ros::Duration> command_waiting_times_;
    ros::Timer command_timer_;
    mavros_msgs::State current_state_;

    bool connected_ = false;
    void secureConnection();
    // CallBacks
    void CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg);
    void TimedCommandCallback(const ros::TimerEvent& e);
    void MultiDofJointTrajectoryCallback(
            const trajectory_msgs::MultiDOFJointTrajectoryConstPtr&
            trajectory_reference_msg);
    void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
    void OdometryCallbackV2(const nav_msgs::OdometryConstPtr &odometry_msg);
    void stateCallBack(const mavros_msgs::State::ConstPtr& msg);
    void dynamicReconfigureCallback(const a_uav_controller::parametersConfig&, const uint32_t);
    double thrust;
    double xToruqe;
    double yToruqe;
public:
    void setThrust(double thrust);
    void setXToruqe(double xToruqe);
    void setYToruqe(double yToruqe);
};



#endif //A_UAV_CONTROLLER_CONTROLLER_NODE_H
