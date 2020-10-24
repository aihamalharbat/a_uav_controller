/**
 * @basic node
 * @brief Offboard control example node, written with MAVROS version 1.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "mavros_msgs/ActuatorControl.h"
#include <mavros_msgs/Thrust.h>


mavros_msgs::State current_state;
float pose_measured[6];

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    if (msg->armed){
        ROS_INFO("ARMED from State_msg.");
    }
    else {
        ROS_INFO("NOT ARMED from State_msg.");
    }

    if (msg->mode == "OFFBOARD"){
        ROS_INFO("OFFBOARD from State_msg.");
    }
    else {
        ROS_INFO("NOT OFFBOARD from State_msg.");
    }
}

void uav_pose_read(const geometry_msgs::PoseStamped pose_msg){
    pose_measured[0] = pose_msg.pose.orientation.x;
    pose_measured[1] = pose_msg.pose.orientation.y;
    pose_measured[2] = pose_msg.pose.orientation.z;
    pose_measured[3] = pose_msg.pose.position.x;
    pose_measured[4] = pose_msg.pose.position.y;
    pose_measured[5] = pose_msg.pose.position.z;
}

float alt_p_controller (const float p_gain, const float setpoint, const float measured){
    float error = setpoint - measured;
    float control = p_gain * error;
    if (control > 1){
        control = 1;
    }
    else if (control < 0){
        control = 0;
    }
    ROS_INFO_ONCE("Control fcn! control = %f", control);
    return control;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Two_offB_Controller");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber uav_pose = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, uav_pose_read);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher attitude_cmd = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);
    ros::Publisher thrust_cmd = nh.advertise<mavros_msgs::Thrust>
            ("mavros/setpoint_attitude/thrust", 10);
    ros::Publisher actuators_cmd = nh.advertise<mavros_msgs::ActuatorControl>
            ("mavros/actuator_control", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    int counter = 0;
    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
//    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    mavros_msgs::ActuatorControl act_cmd;
    act_cmd.group_mix = 0;
    act_cmd.controls[0] = 0;
    act_cmd.controls[1] = 0;
    act_cmd.controls[2] = 0;
    act_cmd.controls[3] = 0;

    mavros_msgs::Thrust thrust_msg;
    thrust_msg.thrust = 0;

    geometry_msgs::PoseStamped attitude_msg;
    attitude_msg.pose.orientation.w = 0;
    attitude_msg.pose.orientation.x = 0;
    attitude_msg.pose.orientation.y = 0;
    attitude_msg.pose.orientation.z = 0;


    //send a few setpoints before starting off-board mode
    for(int i = 100; ros::ok() && i > 0; --i){
//        local_pos_pub.publish(pose);  // publish pose on mavros/setpoint_position/local
        actuators_cmd.publish(act_cmd);
//        attitude_cmd.publish(attitude_msg);
//        thrust_cmd.publish(thrust_msg);
        ros::spinOnce();              //resposible to handle communication events, e.g. arriving messages
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    /*  *********************************
     *              ARM & OFFOARD
     *  *********************************
     */
    ros::Rate slow(0.5);
    while (current_state.mode != "OFFBOARD" && !current_state.armed ){
        if (current_state.mode != "OFFBOARD" &&
        set_mode_client.exists() &&
        set_mode_client.isValid()){
            set_mode_client.call(offb_set_mode);
            ROS_INFO("Offboard enabled");
        }
        if (!current_state.armed &&
        arming_client.exists() &&
        arming_client.isValid()){
            arming_client.call(arm_cmd);
            ROS_INFO("Vehicle armed");
        }
        ros::spinOnce();
//        slow.sleep();
    }
    // record the begining ros time
    ros::Time last_request = ros::Time::now();
    //Check whether it's time to exit.
    while(ros::ok()){

        act_cmd.controls[3] = 0.7 + alt_p_controller(0.2,1,pose_measured[5]);
//        act_cmd.controls[3] = 1;
//        thrust_msg.thrust = 0.7 + alt_p_controller(0.2,1,pose_measured[5]);

//        if (counter < 400 ){
//            pose.pose.position.x = 0;
//            pose.pose.position.y = 0;
//        pose.pose.position.z = 2;
//        } else if (counter >= 400 && counter < 600){
//            pose.pose.position.x = 2;
//            pose.pose.position.y = 0;
//            pose.pose.position.z = 2;
//            //pose.pose.orientation.z = 0.738 ;
//            //pose.pose.orientation.w = - 0.67;
//
//            //tf::createQuaternionMsgFromYaw(1.57);
//        } else if (counter >= 600 && counter < 800) {
//            pose.pose.position.x = 2;
//            pose.pose.position.y = 2;
//            pose.pose.position.z = 2;
//            //pose.pose.orientation.z = 0.738;
//            //pose.pose.orientation.w = 0.065;
//            //pxhwk_att_pub.publish(command_msg);
//        }
//        else if (counter >= 800 && counter < 1000){
//            pose.pose.position.x = 0;
//            pose.pose.position.y = 2;
//            pose.pose.position.z = 2;
//        }
//
//        else if (counter >= 1000 && counter < 1200){
//            pose.pose.position.x = 0;
//            pose.pose.position.y = 0;
//            pose.pose.position.z = 2;
//        }
//
//        else {
//            pose.pose.position.x = 0;
//            pose.pose.position.y = 0;
//            pose.pose.position.z = 0;
//        }
//
//        if(current_state.mode != "OFFBOARD" &&
//           (ros::Time::now() - last_request > ros::Duration(5.0))){
//            if( set_mode_client.call(offb_set_mode) &&
//                offb_set_mode.response.mode_sent){
//                ROS_INFO("Offboard enabled");
//            }
//            last_request = ros::Time::now();
//        } else {
//            ROS_INFO("IN ELSE");
//            if( !current_state.armed &&
//                (ros::Time::now() - last_request > ros::Duration(5.0))){
//                ROS_INFO("IN IF1");
//                if( arming_client.call(arm_cmd) &&
//                    arm_cmd.response.success){
//                    ROS_INFO("Vehicle armed");
//                }
//                last_request = ros::Time::now();
//
//            }
//        }

//        local_pos_pub.publish(pose);    // publish pose on mavros/setpoint_position/local
        actuators_cmd.publish(act_cmd);
//        ROS_INFO("Startposition z=%f", pose.pose.position.z);
        ROS_INFO("current z = %f", pose_measured[5]);
//        ROS_INFO("current cmd = %f",  thrust_msg.thrust);
//        thrust_cmd.publish(thrust_msg);
//        attitude_cmd.publish(attitude_msg);
        counter ++;
        ros::spinOnce();
        rate.sleep();
        //last_request = ros::Time::now();

    }

    return 0;
}

//after 5sec giving command to drone to takeoff to an altitude of 1.5m
//while(ros::ok() && !current_state.armed){

//if( ros::Time::now() - last_request > ros::Duration(5.0)){
//ROS_INFO("change waypoint");

//pose.pose.position.z = 1.5;

//  for(int i = 0; ros::ok() && i < 10*20; ++i){   // sending altitude command continuously for around 3.33min(200sec)
//    local_pos_pub.publish(pose);
//    ros::spinOnce();
//    rate.sleep();
//    }
//    ROS_INFO("go to altitude of z=%f !", pose.pose.position.z);
//  }
//}

/* Notes about the while loop: services called many times until the message arrived about state change.
     * Tests with different msg_type and initialization
     * Worked with initial pose_msgs && pose_control:
            {
                ERROR [tone_alarm] notify negative
                [ INFO] [1602837193.291388089, 15.380000000]: WP: mission received
                [ WARN] [1602837200.013687953, 22.088000000]: CMD: Unexpected command 176, result 0
                INFO  [commander] Armed by external command
                WARN  [mc_pos_control] Failsafe: stop and wait
                INFO  [commander] Takeoff detected
                WARN  [commander] Failsafe enabled: no RC and no offboard
                INFO  [commander] Failsafe mode activated
                WARN  [tone_alarm] battery warning (fast)
                INFO  [commander] Landing detected
                INFO  [commander] Disarmed by landing
                INFO  [logger] closed logfile, bytes written: 2156744

            }
     *  Test without arming and mode: sends messages no problem, nothing happens cz not armed and:
            {
                ERROR [tone_alarm] notify negative
                [ INFO] [1602801518.867410778, 15.228000000]: WP: mission received
                [ WARN] [1602801529.809473018, 26.144000000]: CMD: Unexpected command 176, result 0
                INFO  [commander] Armed by external command
                WARN  [mc_pos_control] Failsafe: stop and wait
                WARN  [commander] Failsafe enabled: no RC and no offboard
                INFO  [commander] Failsafe mode activated
                WARN  [tone_alarm] battery warning (fast)
                INFO  [commander] Failsafe mode deactivated
                INFO  [commander] Takeoff detected
                WARN  [commander] Failsafe enabled: no RC and no offboard
                INFO  [commander] Failsafe mode activated
                INFO  [commander] Landing detected
                INFO  [commander] Disarmed by landing
                INFO  [logger] closed logfile, bytes written: 2855730
            }
     *  Test initial act_cmd && act_control: stops after the first message, and:
            {
                ERROR [tone_alarm] notify negative
                [ INFO] [1602801373.145744956, 15.316000000]: WP: mission received
                [ WARN] [1602801380.470737123, 22.624000000]: CMD: Unexpected command 176, result 0
                INFO  [commander] Armed by external command
                ERROR [sensors] Accel #0 fail:  TIMEOUT!
                ERROR [vehicle_air_data] BARO #0 failed:  TIMEOUT!
                ERROR [vehicle_magnetometer] MAG #0 failed:  TIMEOUT!
                WARN  [commander] Failsafe enabled: no RC and no offboard
                INFO  ERROR [tone_alarm] notify negative
                [commander] Failsafe mode activated
                WARN  [tone_alarm] battery warning (fast)
                INFO  [commander] Failsafe mode deactivated
                INFO  [commander] Takeoff detected
                WARN  [commander] Failsafe enabled: no RC and no offboard
                INFO  [commander] Failsafe mode activated
                WARN  [commander] Connection to mission computer lost
                INFO  [commander] Landing detected
                INFO  [commander] Disarmed by landing
                INFO  [logger] closed logfile, bytes written: 1209284
            }
            -- worked Oct16 1038--
                ERROR [tone_alarm] notify negative
                [ INFO] [1602837388.641015208, 15.356000000]: WP: mission received
                [ WARN] [1602837395.292530707, 21.992000000]: CMD: Unexpected command 176, result 0
                INFO  [commander] Armed by external command
                INFO  [commander] Disarmed by auto preflight disarming
                INFO  [logger] closed logfile, bytes written: 1668791

     *  Test while loop with thrust_msgs attitude_msg && initial thrust_msgs attitude_msg:
            does not stop after first message, but no reaction from UAV, and:
            {
                ERROR [tone_alarm] notify negative
                [ INFO] [1602801058.645255566, 15.416000000]: WP: mission received
                ERROR [tone_alarm] notify negative
                [ WARN] [1602801076.162872323, 32.892000000]: CMD: Unexpected command 176, result 1
                INFO  [commander] Armed by external command
                WARN  [commander] Failsafe enabled: No manual control stick input
                INFO  [commander] Failsafe mode activated
                INFO  [navigator] RTL HOME activated
                INFO  [navigator] RTL: landing at home position.
                INFO  [commander] Failsafe mode deactivated
                WARN  [tone_alarm] battery warning (fast)
                INFO  [commander] Disarmed by auto preflight disarming
                INFO  [logger] closed logfile, bytes written: 2260243
            }
    */