//
// Created by aalharbat on 14-10-20.
//

#include "../include/a_uav_controller/controller.h"
#include "ros/ros.h"


controller::controller() : controller_active_(false) {
    getParameters();
}

void controller::computeAllocationMatrix() {
    const double kDegToRad = M_PI / 180.0;
    Eigen::MatrixXd A;

    // Set the allocation matrix A.
    Eigen::Vector4d k;  // Helper diagonal matrix.
    k << _thrust_constant, _thrust_constant * _uav_arm_length,
            _thrust_constant * _uav_arm_length, _moment_constant * _thrust_constant;
    const double kS = std::sin(30 * kDegToRad);
    const double kC = std::cos(30 * kDegToRad);
    A.resize(4, 6);
    A << 1, 1, 1, 1, 1, 1,
            kS, 1, kS, -kS, -1, -kS,
            -kC, 0, kC, kC, 0, -kC,
            -1, 1, -1, 1, -1, 1;
    A = k.asDiagonal() * A;

    // Compute the pseudo-inverse of A.
    // This matrix maps thrust and torque commands to squared rotor speeds.
    thrust_and_torque_to_rotor_velocities_.resize(6, 4);
    thrust_and_torque_to_rotor_velocities_ =
            A.transpose() * (A * A.transpose()).inverse();
    //ENDRM
}

void controller::calculateRotorVelocities(
        Eigen::VectorXd* rotor_velocities) const {
    assert(rotor_velocities);

    rotor_velocities->resize(6);
    // Return 0 velocities on all rotors, until the first command is received.
    if (!controller_active_) {
        rotor_velocities->setZero();
        return;
    }

    // Trajectory tracking.
    double T;
    Eigen::Vector3d B_z_d;
    computeTrajectoryTracking(&T, &B_z_d);
    ROS_INFO("(calculateRotorVelocities)Thrust = %f", T);

    // Attitude tracking.
    Eigen::Vector3d tau;
    computeAttitudeTracking(B_z_d, &tau);

    // Compute rotor speeds with allocation inverse.
    Eigen::Vector4d thrust_torque;
    thrust_torque << T, tau;

    *rotor_velocities = thrust_and_torque_to_rotor_velocities_ * thrust_torque;
    *rotor_velocities = rotor_velocities->cwiseMax(
            Eigen::VectorXd::Zero(rotor_velocities->rows()));
    *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void controller::calculateActCmds(
        Eigen::VectorXd *ActCmds) {
    assert(ActCmds);

    ActCmds->resize(4);
    // Return 0 velocities on all rotors, until the first command is received.
    if (!controller_active_) {
        ActCmds->setZero();
        return;
    }

    // Trajectory tracking.
    double T;
    Eigen::Vector3d B_z_d;
    computeTrajectoryTracking(&T, &B_z_d);
    ROS_INFO("(calculateActCmds)Thrust = %f", T);

    // Attitude tracking.
    Eigen::Vector3d tau;
    computeAttitudeTracking(B_z_d, &tau);

    // Compute rotor speeds with allocation inverse.
    // NOT DONE HERE! -AYHAM
    Eigen::Vector4d torque_thrust;
    torque_thrust << tau, T;
    Eigen::Vector4d normalized_torque_thrust;
    normalized_torque_thrust = normalizeActCmds(&torque_thrust);
    *ActCmds = normalized_torque_thrust;
}


void controller::setOdometry(const mav_msgs::EigenOdometry &odometry) {
    odometry_ = odometry;
}

void controller::setTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint &command_trajectory) {
    command_trajectory_ = command_trajectory;
    controller_active_ = true;
}

void controller::computeTrajectoryTracking(double *T, Eigen::Vector3d *B_z_d) const {
    assert(T);
    assert(B_z_d);

    // Transform velocity to world frame.
    const Eigen::Matrix3d R_I_B = odometry_.orientation_W_B.toRotationMatrix();
    const Eigen::Vector3d I_v = R_I_B * odometry_.velocity_B;

    // Compute translational tracking errors.
    const Eigen::Vector3d e_p =
            (odometry_.position_W - initial_position) - command_trajectory_.position_W;
    const Eigen::Vector3d e_v = I_v - command_trajectory_.velocity_W;
    Eigen::Vector3d I_a_ref = command_trajectory_.acceleration_W;
    ROS_INFO_ONCE("[controller] Z_ERROR: %f", e_p[2]);

    // Compute the the desired acceleration in inertial
    // coordinates.
    const Eigen::Vector3d I_a_d = -kPositionGain.cwiseProduct(e_p) -
                                  kVelocityGain.cwiseProduct(e_v) +
                                  _gravity * Eigen::Vector3d::UnitZ() + I_a_ref;

    // Compute the control thrust T by projecting the
    // acceleration onto the body z-axis.
    *T = _uav_mass * I_a_d.dot(R_I_B.col(2));

    // Compute the desired thrust direction B_z_d from the
    // desired acceleration I_a_d.
    *B_z_d = I_a_d;
    B_z_d->normalize();
}

void controller::computeAttitudeTracking(const Eigen::Vector3d &B_z_d, Eigen::Vector3d *tau) const {
    assert(tau);

    const Eigen::Matrix3d R_IB = odometry_.orientation_W_B.toRotationMatrix();

    // Compute the the desired heading direction B_x_d from
    // the commanded yaw.
    const double yaw_ref = command_trajectory_.getYaw();
    const Eigen::Vector3d B_x_d(std::cos(yaw_ref), std::sin(yaw_ref), 0.0);


    // Compute B_y_d which is perpendicular to B_z_d and
    // B_x_d.
    Eigen::Vector3d B_y_d = B_z_d.cross(B_x_d);
    B_y_d.normalize();

    // Compute the desired attitude R_IB_d.
    Eigen::Matrix3d R_IB_d;
    R_IB_d.col(0) = B_y_d.cross(B_z_d);
    R_IB_d.col(1) = B_y_d;
    R_IB_d.col(2) = B_z_d;

    // Compute the attitude tracking errors e_R and e_omega.
    const Eigen::Matrix3d e_R_matrix =
            0.5 * (R_IB_d.transpose() * R_IB - R_IB.transpose() * R_IB_d);
    Eigen::Vector3d e_R;
    vectorFromSkewMatrix(e_R_matrix, &e_R);

    const Eigen::Vector3d omega_ref =
            command_trajectory_.getYawRate() * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d omega = odometry_.angular_velocity_B;
    const Eigen::Vector3d e_omega = omega - R_IB.transpose() * R_IB_d * omega_ref;

    //Compute the command torque.
    *tau = -kAttitudeGain.cwiseProduct(e_R)
            - kAngularRateGain.cwiseProduct(e_omega)
            + omega.cross(_inertia_matrix.asDiagonal() * omega);
}

Eigen::Vector4d controller::normalizeActCmds(Eigen::Vector4d *wrench) {
    //  Assuming that:
    //  wrench = [roll_torque; pitch_torque; yaw_torque; thrust]
    Eigen::Vector4d normalizedWrench;
//      These values are form this matlab code:
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

void controller::getParameters(){
    ros::NodeHandle priv_nh("~");
    if (!priv_nh.getParam("uav_parameters/mass", _uav_mass)){
        ROS_ERROR("Could not find topic  parameter!");
    }
    if (!priv_nh.getParam("uav_parameters/inertia/x", _inertia_matrix[0])){
        ROS_ERROR("Could not find topic  parameter!");
    }
    if (!priv_nh.getParam("uav_parameters/inertia/y", _inertia_matrix[1])){
        ROS_ERROR("Could not find topic  parameter!");
    }
    if (!priv_nh.getParam("uav_parameters/inertia/z", _inertia_matrix[2])){
        ROS_ERROR("Could not find topic  parameter!");
    }
    if (!priv_nh.getParam("uav_parameters/gravity", _gravity)){
        ROS_ERROR("Could not find topic  parameter!");
    }
    if (!priv_nh.getParam("uav_parameters/max_roll_torque", _max_roll_torque)){
        ROS_ERROR("Could not find topic  parameter!");
    }
    if (!priv_nh.getParam("uav_parameters/max_pitch_torque", _max_pitch_torque)){
        ROS_ERROR("Could not find topic  parameter!");
    }
    if (!priv_nh.getParam("uav_parameters/max_yaw_torque", _max_yaw_torque)){
        ROS_ERROR("Could not find topic  parameter!");
    }
    if (!priv_nh.getParam("uav_parameters/max_thrust", _max_thrust)){
        ROS_ERROR("Could not find topic  parameter!");
    }
}

void controller::setKPositionGain(const Eigen::Vector3d &kPositionGain) {
    controller::kPositionGain = kPositionGain;
}

void controller::setKVelocityGain(const Eigen::Vector3d &kVelocityGain) {
    controller::kVelocityGain = kVelocityGain;
}

void controller::setKAttitudeGain(const Eigen::Vector3d &kAttitudeGain) {
    controller::kAttitudeGain = kAttitudeGain;
}

void controller::setKAngularRateGain(const Eigen::Vector3d &kAngularRateGain) {
    controller::kAngularRateGain = kAngularRateGain;
}

void controller::setInitialPosition(const Eigen::Vector3d &initialPosition) {
    initial_position = initialPosition;
}