//
// Created by aalharbat on 14-10-20.
//

#ifndef A_UAV_CONTROLLER_CONTROLLER_H
#define A_UAV_CONTROLLER_CONTROLLER_H

#include <mav_msgs/eigen_mav_msgs.h>
#include <Eigen/Eigen>

class controller {
public:
    controller();
    void computeAllocationMatrix();
    void calculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;
    void calculateActCmds(Eigen::VectorXd* ActCmds) const;
    void setOdometry(const mav_msgs::EigenOdometry& odometry);
    void setTrajectoryPoint(
            const mav_msgs::EigenTrajectoryPoint& command_trajectory);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    bool controller_active_;
    Eigen::MatrixX4d thrust_and_torque_to_rotor_velocities_;

    mav_msgs::EigenTrajectoryPoint command_trajectory_;
    mav_msgs::EigenOdometry odometry_;

    void computeTrajectoryTracking(double* T, Eigen::Vector3d* B_z_d) const;
    void computeAttitudeTracking(const Eigen::Vector3d& B_z_d,
                                 Eigen::Vector3d* tau) const;


};

// Functions for the controller
inline void skewMatrixFromVector(const Eigen::Vector3d& vector,
                                 Eigen::Matrix3d* skew_matrix) {
    *skew_matrix << 0, -vector.z(), vector.y(), vector.z(), 0, -vector.x(),
            -vector.y(), vector.x(), 0;
}

inline void vectorFromSkewMatrix(const Eigen::Matrix3d& skew_matrix,
                                 Eigen::Vector3d* vector) {
    *vector << skew_matrix(2, 1), skew_matrix(0, 2), skew_matrix(1, 0);
}

#endif //A_UAV_CONTROLLER_CONTROLLER_H
