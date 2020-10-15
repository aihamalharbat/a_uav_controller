//
// Created by aalharbat on 14-10-20.
//

#include "../include/a_uav_controller/controller.h"
#include "ros/ros.h"

// MAV parameters.
const double kMass = 1.725;
const double kArmLength = 0.343;    // xyz = 0.13 -0.22 0.023
const Eigen::Vector3d kInertiaDiag =
        Eigen::Vector3d(0.029125, 0.029125, 0.055225);
const double kMomentConstant = 1.6e-2;
const double kThrustConstant = 8.568e-6;
const double kGravity = 9.81;

// Default values for the lee position controller and the Asctec Firefly.
const Eigen::Vector3d kPositionGain = Eigen::Vector3d(6, 6, 6) / kMass;
const Eigen::Vector3d kVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7) / kMass;
const Eigen::Vector3d kAttitudeGain = Eigen::Vector3d(3, 3, 0.035);
const Eigen::Vector3d kAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.025);

controller::controller() : controller_active_(false) {

}

void controller::computeAllocationMatrix() {
    const double kDegToRad = M_PI / 180.0;
    Eigen::MatrixXd A;
    //STARTUNCOMMENT
    // TASK (Ex. 1.3): Set the allocation matrix A.
    // Hint: A Tux Eigen matrix can be allocated as follows:
    // Eigen::MatrixXd mat;
    // mat.resize(2, 3)
    // mat << 1, 2, 3,
    //        4, 5, 6;
    // Furthermore use std::sin(alpha), std::cos(alpha), kArmLength,
    // kThrustConstant, kMomentConstant.
    // A.resize(TODO);
    // A << TODO
    //ENDUNCOMMENT

    //STARTRM
    Eigen::Vector4d k;  // Helper diagonal matrix.
    k << kThrustConstant, kThrustConstant * kArmLength,
            kThrustConstant * kArmLength, kMomentConstant * kThrustConstant;
    const double kS = std::sin(30 * kDegToRad);
    const double kC = std::cos(30 * kDegToRad);
    A.resize(4, 6);
    A << 1, 1, 1, 1, 1, 1,
            kS, 1, kS, -kS, -1, -kS,
            -kC, 0, kC, kC, 0, -kC,
            -1, 1, -1, 1, -1, 1;
    A = k.asDiagonal() * A;
    //ENDRM

    //STARTUNCOMMENT
    // TASK (Ex. 1.3): Compute the pseudo-inverse of A.
    // This matrix maps thrust and torque commands to squared rotor speeds.
    // Hint: Use mat.transpose() and mat.inverse().
    // thrust_and_torque_to_rotor_velocities_.resize(TODO);
    // thrust_and_torque_to_rotor_velocities_ = TODO
    //ENDUNCOMMENT
    //STARTRM
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
        Eigen::VectorXd* ActCmds) const {
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
    Eigen::Vector4d torque_thrust;
    torque_thrust << tau, T;

    *ActCmds = torque_thrust;
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
            odometry_.position_W - command_trajectory_.position_W;
    const Eigen::Vector3d e_v = I_v - command_trajectory_.velocity_W;
    Eigen::Vector3d I_a_ref = command_trajectory_.acceleration_W;

    //STARTUNCOMMENT
    // TASK (Ex. 1.4.1): Compute the the desired acceleration in inertial
    // coordinates.
    // Hint: Use a.cwiseProduct(b) and the constants defined on top, i.e.,
    // kPositionGain, kVelocityGain, and kGravity.
    // Eigen::Vector3d I_a_d = TODO
    //ENDUNCOMMENT
    //STARTRM

    const Eigen::Vector3d I_a_d = -kPositionGain.cwiseProduct(e_p) -
                                  kVelocityGain.cwiseProduct(e_v) +
                                  kGravity * Eigen::Vector3d::UnitZ() + I_a_ref;
    //ENDRM

    //STARTUNCOMMENT
    // TASK (Ex. 1.4.1): Compute the control thrust T by projecting the
    // acceleration onto the body z-axis.
    // Hint: Use kMass, I_a_d, and R_I_B.
    // *T = TODO
    //ENDUNCOMMENT
    //STARTRM
    *T = kMass * I_a_d.dot(R_I_B.col(2));
    //ENDRM

    //STARTUNCOMMENT
    // TASK (Ex. 1.4.1): Compute the desired thrust direction B_z_d from the
    // desired acceleration I_a_d.
    // *B_z_d = TODO
    // Hint: Use .normalize() or .norm() to make a vector unitlength.
    //ENDUNCOMMENT
    //STARTRM
    *B_z_d = I_a_d;
    B_z_d->normalize();
    //ENDRM
}

void controller::computeAttitudeTracking(const Eigen::Vector3d &B_z_d, Eigen::Vector3d *tau) const {
    assert(tau);

    const Eigen::Matrix3d R_IB = odometry_.orientation_W_B.toRotationMatrix();

    //STARTUNCOMMENT
    // TASK (Ex. 1.4.2): Compute the the desired heading direction B_x_d from
    // the commanded yaw.
    // const double yaw_ref = command_trajectory_.getYaw();
    // Eigen::Vector3d B_x_d(TODO);
    // Hint: Use std::sin(double) and std::cos(double).
    //ENDUNCOMMENT
    //STARTRM
    const double yaw_ref = command_trajectory_.getYaw();
    const Eigen::Vector3d B_x_d(std::cos(yaw_ref), std::sin(yaw_ref), 0.0);
    //ENDRM

    //STARTUNCOMMENT
    // TASK (Ex. 1.4.2): Compute B_y_d which is perpendicular to B_z_d and
    // B_x_d.
    //
    // Eigen::Vector3d B_y_d = TODO
    // Hint: Use a.cross(b) and .normalize() or .norm().
    //ENDUNCOMMENT
    //STARTRM
    Eigen::Vector3d B_y_d = B_z_d.cross(B_x_d);
    B_y_d.normalize();
    //ENDRM

    //STARTUNCOMMENT
    // TASK (Ex. 1.4.2): Compute the desired attitude R_IB_d.
    //
    // Hint: Use the given equation.
    // Eigen::Matrix3d R_IB_d;
    // Eigen::Matrix3d R_IB_d.col(0) = TODO
    // Eigen::Matrix3d R_IB_d.col(1) = TODO
    // Eigen::Matrix3d R_IB_d.col(2) = TODO
    //ENDUNCOMMENT
    //STARTRM
    Eigen::Matrix3d R_IB_d;
    R_IB_d.col(0) = B_y_d.cross(B_z_d);
    R_IB_d.col(1) = B_y_d;
    R_IB_d.col(2) = B_z_d;
    //ENDRM

    //STARTUNCOMMENT
    // TASK (Ex. 1.4.2): Compute the attitude tracking errors e_R and e_omega.
    //
    // Hint: Use vectorFromSkewMatrix(a, &b), where a is the skrew matrix and b is
    // the result vector.
    // Eigen::Matrix3d e_R_matrix = TODO
    // Eigen::Vector3d e_R = TODO;
    //
    // const Eigen::Vector3d omega_ref = command_trajectory_.getYawRate() *
    // Eigen::Vector3d::UnitZ();
    // const Eigen::Vector3d omega = odometry_.angular_velocity;
    // Eigen::Vector3d e_omega = TODO
    //ENDUNCOMMENT
    //STARTRM
    const Eigen::Matrix3d e_R_matrix =
            0.5 * (R_IB_d.transpose() * R_IB - R_IB.transpose() * R_IB_d);
    Eigen::Vector3d e_R;
    vectorFromSkewMatrix(e_R_matrix, &e_R);

    const Eigen::Vector3d omega_ref =
            command_trajectory_.getYawRate() * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d omega = odometry_.angular_velocity_B;
    const Eigen::Vector3d e_omega = omega - R_IB.transpose() * R_IB_d * omega_ref;
    //ENDRM

    //STARTUNCOMMENT
    // TASK (Ex. 1.4.2): Compute the command torque.
    //
    // Hint: Use a.cwiseProduct(b) and kInertiaDiag.asDiagonal().
    // *tau = TODO
    //ENDUNCOMMENT
    //STARTRM
    *tau = -kAttitudeGain.cwiseProduct(e_R) -
           kAngularRateGain.cwiseProduct(e_omega) +
           omega.cross(kInertiaDiag.asDiagonal() * omega);
    //ENDRM
}





