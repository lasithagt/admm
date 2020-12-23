
#include "KukaModel.h"



KUKAModelKDL::~KUKAModelKDL()
{
}

int KUKAModelKDL::initRobot() 
{

    return true;
}

void KUKAModelKDL::getForwardKinematics(double* q, double* qd, double *qdd, Eigen::Matrix<double,3,3>& poseM, Eigen::Vector3d& poseP, Eigen::Vector3d& vel, Eigen::Vector3d& accel, bool computeOther)
{
 
}

/* given q, qdot, qddot, outputs torque output*/
void KUKAModelKDL::getInverseDynamics(double* q, double* qd, double* qdd, Eigen::VectorXd& torque)
{

}

void KUKAModelKDL::getForwardDynamics(double* q, double* qd, const Eigen::VectorXd& force_ext, Eigen::VectorXd& qdd)
{


}

void KUKAModelKDL::getMassMatrix(double* q, Eigen::MatrixXd& massMatrix)
{

} 

void KUKAModelKDL::getCoriolisMatrix(double* q, double* qd, Eigen::VectorXd& coriolis) // change
{

}
 

void KUKAModelKDL::getGravityVector(double* q, Eigen::VectorXd& gravityTorque)
{

}
 

void KUKAModelKDL::getSpatialJacobian(double* q, Eigen::MatrixXd& jacobian)
{

} 

void KUKAModelKDL::getSpatialJacobianDot(double* q, double* qd, Eigen::MatrixXd& jacobianDot)
{   

}

void KUKAModelKDL::ik()
{
    
}




