#ifndef KUKA_MODEL_H
#define KUKA_MODEL_H


#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfdsolver_recursive_newton_euler.hpp>
#include <kdl/chainfdsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

#include <memory>
#include <string.h>
#include <iostream>
#include <algorithm>

#include "models.h"
#include "RobotAbstract.h"


struct KUKAModelKDLInternalData : RobotAbstractInternalData
{
    int numJoints;
    Eigen::MatrixXd Kv; // joint dynamic coefficient
    Eigen::MatrixXd Kp;
};

class KUKAModelKDL : public RobotAbstract
{

public:
    KUKAModelKDL(const KDL::Chain& robotChain, const KUKAModelKDLInternalData& robotParams);

    ~KUKAModelKDL();

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

    KUKAModelKDLInternalData robotParams_;

private:
    // patch variables for speed
    KDL::JntArray q_;   
    KDL::JntArray qd_;   
    KDL::JntArray qdd_;
    KDL::JntSpaceInertiaMatrix inertia_mat_;     // Interia Matrix
    KDL::JntArray coriolis_;                     // CoriolisVector
    KDL::JntArray gravity_;                      // GravityVector
    KDL::Jacobian jacobian_;
    KDL::Frame frame_;
    KDL::FrameVel frame_vel_;

    Eigen::Matrix<double, 7, 7> Kv_;
    KDL::ChainDynParam* dynamicsChain_;
    KDL::Chain robotChain_;

};

#endif // KUKA_MODEL_HPP
