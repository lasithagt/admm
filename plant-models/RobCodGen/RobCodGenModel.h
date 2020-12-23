#ifndef KUKA_MODEL_ROBCODGEN_H
#define KUKA_MODEL_ROBCODGEN_H

#include <iostream>

#include <Eigen/Dense>
#include <algorithm>

#include <memory>
#include <string.h>

#include "models.h"
#include "RobotAbstract.h"

#include <ct/rbd/rbd.h>

#include <memory>
#include <array>

#include <gtest/gtest.h>

#include <ct/rbd/systems/linear/RbdLinearizer.h>
#include "ct/rbd/systems/FixBaseFDSystem.h"
#include "ct/rbd/systems/FloatingBaseFDSystem.h"


#include <ct/rbd/rbd.h>


struct RobCodGenModelInternalData : RobotAbstractInternalData
{
    int numJoints;
    Eigen::MatrixXd Kv; // joint dynamic coefficient
    Eigen::MatrixXd Kp;
};

class RobCodGenModel : public RobotAbstract
{

public:
    RobCodGenModel(const KDL::Chain& robotChain, const RobCodGenModelInternalData& robotParams);

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
    // patch variables for speed

};

#endif // KUKA_MODEL_HPP
