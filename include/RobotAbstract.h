#ifndef ROBOT_ABSTRACT_H
#define ROBOT_ABSTRACT_H

#include <Eigen/Dense>

/* structure for the active robot */
struct RobotAbstractInternalData
{

};

/* abstract class for a serial robot */
class RobotAbstract
{	
public:
	// struct RobotAbstractInternalData* m_data;

	RobotAbstract() = default;
	virtual ~RobotAbstract(){}

	virtual int initRobot() = 0;

	virtual void getForwardKinematics(double* q, double* qd, double *qdd, Eigen::Matrix<double,3,3>& poseM, Eigen::Vector3d& poseP, Eigen::Vector3d& vel, Eigen::Vector3d& accel, bool computeOther) = 0;

	/* given q, qdot, qddot, outputs torque output*/
	virtual void getInverseDynamics(double* q, double* qd, double* qdd, Eigen::VectorXd& torque) = 0;

	virtual void getForwardDynamics(double* q, double* qd, const Eigen::VectorXd& force_ext, Eigen::VectorXd& qdd) = 0;

	virtual void getForceTorque() {};

	virtual void getMassMatrix(double* jointPositions, Eigen::MatrixXd& massMatrix) = 0;

	virtual void getCoriolisMatrix(double* q, double* qd, Eigen::VectorXd& coriolis) = 0;

	virtual void getGravityVector(double* q, Eigen::VectorXd& gravityTorque) = 0;

	virtual void getSpatialJacobian(double* q, Eigen::MatrixXd& jacobian) = 0;

	void getSpatialJacobianDot(double* q, double* qd, Eigen::MatrixXd& jacobianDot);

	void ik();
};
#endif  //ROBOT_ABSTRACT_H
