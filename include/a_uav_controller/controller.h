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
    void calculateActCmds(Eigen::VectorXd *ActCmds);
    void setOdometry(const mav_msgs::EigenOdometry& odometry);
    void setTrajectoryPoint(
            const mav_msgs::EigenTrajectoryPoint& command_trajectory);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    bool controller_active_;
    // UAV Parameter
    double kMass;
    double kArmLength;
    Eigen::Vector3d kInertiaDiag;
    double kMomentConstant;
    double kThrustConstant;
    double kGravity;
    // Controller Parameters
    Eigen::Vector3d kPositionGain;
    Eigen::Vector3d kVelocityGain;
    Eigen::Vector3d kAttitudeGain;
    Eigen::Vector3d kAngularRateGain;
    Eigen::MatrixX4d thrust_and_torque_to_rotor_velocities_;
    mav_msgs::EigenTrajectoryPoint command_trajectory_;
    mav_msgs::EigenOdometry odometry_;
    Eigen::Vector4d normalizeActCmds (Eigen::Vector4d *wrench);
    void computeTrajectoryTracking(double* T, Eigen::Vector3d* B_z_d) const;
    void computeAttitudeTracking(const Eigen::Vector3d& B_z_d,
                                 Eigen::Vector3d* tau) const;


};

// Math Functions for the controller
inline void skewMatrixFromVector(const Eigen::Vector3d& vector,
                                 Eigen::Matrix3d* skew_matrix) {
    *skew_matrix << 0, -vector.z(), vector.y(),
                    vector.z(), 0, -vector.x(),
                    -vector.y(), vector.x(), 0;
}

inline void vectorFromSkewMatrix(const Eigen::Matrix3d& skew_matrix,
                                 Eigen::Vector3d* vector) {
    *vector << skew_matrix(2, 1), skew_matrix(0, 2), skew_matrix(1, 0);
}

inline Eigen::Vector3d matrix_hat_inv(const Eigen::Matrix3d &m) {      // map from so(3) to R^3
    Eigen::Vector3d v;
    // TODO: Sanity checks if m is skew symmetric
    v << m(7), m(2), m(3);
    return v;
}

inline Eigen::Matrix3d matrix_hat(const Eigen::Vector3d &v) {
    Eigen::Matrix3d m;
    // Sanity checks on M
    m << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
    return m;
}

inline Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q) {
    Eigen::Matrix3d rotmat;
    rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
            2 * q(0) * q(2) + 2 * q(1) * q(3),

            2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
            2 * q(2) * q(3) - 2 * q(0) * q(1),

            2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
            q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    return rotmat;
}

inline Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) {
    Eigen::Vector4d quat;
    quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
            p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
    return quat;
}

inline Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R) {
    Eigen::Vector4d quat;
    double tr = R.trace();
    if (tr > 0.0) {
        double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
        quat(0) = 0.25 * S;
        quat(1) = (R(2, 1) - R(1, 2)) / S;
        quat(2) = (R(0, 2) - R(2, 0)) / S;
        quat(3) = (R(1, 0) - R(0, 1)) / S;
    } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
        double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
        quat(0) = (R(2, 1) - R(1, 2)) / S;
        quat(1) = 0.25 * S;
        quat(2) = (R(0, 1) + R(1, 0)) / S;
        quat(3) = (R(0, 2) + R(2, 0)) / S;
    } else if (R(1, 1) > R(2, 2)) {
        double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
        quat(0) = (R(0, 2) - R(2, 0)) / S;
        quat(1) = (R(0, 1) + R(1, 0)) / S;
        quat(2) = 0.25 * S;
        quat(3) = (R(1, 2) + R(2, 1)) / S;
    } else {
        double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
        quat(0) = (R(1, 0) - R(0, 1)) / S;
        quat(1) = (R(0, 2) + R(2, 0)) / S;
        quat(2) = (R(1, 2) + R(2, 1)) / S;
        quat(3) = 0.25 * S;
    }
    return quat;
}

#endif //A_UAV_CONTROLLER_CONTROLLER_H
