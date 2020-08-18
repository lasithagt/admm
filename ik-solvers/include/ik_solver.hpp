#ifndef IK_SOLVER_HPP
#define IK_SOLVER_HPP

#include <iostream>
#include <Eigen/Dense>
#include "modern_robotics.h"

#define NDOF 7


class IK {
public:
	IK() = default;
	~IK() = default;

	/* Get IK solution given the Td and initial thetalist */
	virtual void getIK(const Eigen::MatrixXd& Td, const Eigen::VectorXd& thetalist0, const Eigen::VectorXd& thetalistd0, const Eigen::VectorXd& q_bar, const Eigen::VectorXd& qd_bar,
	 bool initial, const Eigen::VectorXd& rho, Eigen::VectorXd* thetalist) = 0;

private:
	/* Null space projection */
	virtual void getRedundancyResolution(const Eigen::VectorXd& thetalist, Eigen::VectorXd* q_grad_ret) = 0;

	/* Compute jacobian dot, given the jacobian */
	virtual void getJacobianDot(const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& thetalist, Eigen::MatrixXd* jacobianDot) {};


protected:
	Eigen::MatrixXd Slist;
	Eigen::MatrixXd M;
	double ev;
	double eomg;
};

/* First order IK solution */
class IK_FIRST_ORDER : public IK {

public:
	IK_FIRST_ORDER() {}

	IK_FIRST_ORDER(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& M, const Eigen::MatrixXd& joint_limits, const double& eomg, const double& ev, const Eigen::VectorXd& rho);

	void getIK(const Eigen::MatrixXd& Td, const Eigen::VectorXd& thetalist0, const Eigen::VectorXd& thetalistd0, const Eigen::VectorXd& q_bar, const Eigen::VectorXd& qd_bar, 
		bool initial, const Eigen::VectorXd& rho, Eigen::VectorXd* thetalist);
	
	void getRedundancyResolution(const Eigen::VectorXd& thetalist, Eigen::VectorXd* q_grad_ret);
	
	int maxIterations;
	Eigen::Matrix<double, 4, 4> Tsb;
	Eigen::Vector<double, 6> Vs;
	Eigen::VectorXd rho;
	Eigen::Matrix<double, 2, NDOF> joint_limits;
	Eigen::Vector<double, NDOF> q_range;
	Eigen::Vector<double, NDOF> q_mid;

	Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double, 6, NDOF> > cod;

};


/* Second order IK solution */
class IK_SECOND_ORDER : public IK {

public:
	IK_SECOND_ORDER();

	IK_SECOND_ORDER(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& M, const Eigen::VectorXd& joint_limits, const double& eomg, const double& ev, const Eigen::VectorXd& rho);

};

#endif //IK_SOLVER_HPP

