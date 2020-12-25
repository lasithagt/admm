#ifndef KUKA_MODEL_ROBCODGEN_H
#define KUKA_MODEL_ROBCODGEN_H

#include <iostream>

#include <Eigen/Dense>
#include <algorithm>

#include <string.h>

#include "models.h"
#include "RobotAbstract.h"

#include <ct/rbd/rbd.h>

#include <memory>
#include <array>

// #include <gtest/gtest.h>

#include <ct/rbd/systems/linear/RbdLinearizer.h>
#include "ct/rbd/systems/FixBaseFDSystem.h"
#include "ct/rbd/systems/FloatingBaseFDSystem.h"


#include "KUKA.h"
#include "ct/rbd/robot/Dynamics.h"
#include "ct/rbd/robot/Kinematics.h"

using namespace ct::rbd;


struct RobCodGenModelInternalData : RobotAbstractInternalData
{
    int numJoints;
    Eigen::MatrixXd Kv; // joint dynamic coefficient
    Eigen::MatrixXd Kp;
};

class RobCodGenModel : public RobotAbstract
{

    using control_vector_t = typename KUKA::Dynamics::control_vector_t;
    using ForceVector_t = typename KUKA::Dynamics::ForceVector_t;
    using JointState_t = typename KUKA::Dynamics::JointState_t;
    using JointAcceleration_t = typename KUKA::Dynamics::JointAcceleration_t;
    using ExtLinkForces_t = typename KUKA::Dynamics::ExtLinkForces_t;

public:
    RobCodGenModel();
    ~RobCodGenModel();
    int initRobot();
    void getForwardKinematics(double* q, double* qd, double *qdd, Eigen::Matrix<double,3,3>& poseM, Eigen::Vector3d& poseP, Eigen::Vector3d& vel, Eigen::Vector3d& accel, bool computeOther);
    /* given q, qdot, qddot, outputs torque output*/
    void getInverseDynamics(double* q, double* qd, double* qdd, Eigen::VectorXd& torque);
    void getForwardDynamics(double* q, double* qd, const Eigen::VectorXd& force_ext, Eigen::VectorXd& qdd);
    void getMassMatrix(double* q, Eigen::MatrixXd& massMatrix);
    void getCoriolisMatrix(double* q, double* qd, Eigen::VectorXd& coriolis); // change
    void getGravityVector(double* q, Eigen::VectorXd& gravityTorque);
    void getSpatialJacobian(double* q, Eigen::MatrixXd& jacobian);
    void getSpatialJacobianDot(double* q, double* qd, Eigen::MatrixXd& jacobianDot);
    void ik();

    RobCodGenModelInternalData robotParams_;

private:
    KUKA::Kinematics m_kyn;
    KUKA::Dynamics m_dyn;
    std::shared_ptr<KUKA::Kinematics> kyn_ptr;

    Eigen::Matrix<double, 2*KUKA::Kinematics::NJOINTS, 1>& robot_state;

    // fd
    control_vector_t torque_u;
    ExtLinkForces_t ext_forces;
    JointAcceleration_t id_qdd;
    JointAcceleration_t qdd;

    // velocities and position
    JointState<KUKA::Kinematics::NJOINTS> joint_state;
    EndEffector<KUKA::Kinematics::NJOINTS> robot_ee;
    RBDState<KUKA::Kinematics::NJOINTS> RBD;
    RigidBodyState base_state;
    RigidBodyPose base_pose;
    RigidBodyPose ee_pose;
    kindr::Velocity3D vel;

    // jacobian
    KUKA::Kinematics::Jacobian jac;

    // gravity
    Eigen::VectorXd gravity_u;


};

#endif // KUKA_MODEL_HPP
