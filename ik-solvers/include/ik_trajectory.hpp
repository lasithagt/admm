#ifndef IK_TRAJECTORY_HPP
#define IK_TRAJECTORY_HPP

#include <iostream>
#include <Eigen/Dense>
#include "modern_robotics.h"
#include <cmath>
#include <functional>
#define NDOF 7



/* Template class. First order IK solution for a given SE(3) trajectory */
template<class IK_solver>
class IKTrajectory {

public:

	// data structure for IK options
	struct IKopt {
		IKopt(int NDOF_) : NDOFS(NDOF_) {
			joint_limits.resize(2, NDOFS);
			Slist.resize(6, NDOFS);
		}

		double ev;
		double eomg;
		int NDOFS;
		Eigen::MatrixXd joint_limits;
		Eigen::MatrixXd Slist;
		Eigen::Matrix<double, 4, 4> M;
	};

	IKTrajectory() {}

	~IKTrajectory() {}

	IKTrajectory(const Eigen::MatrixXd& S, const Eigen::MatrixXd& M_, const Eigen::MatrixXd& joint_limits,
	 const double& eomg_, const double& ev_, const Eigen::VectorXd& rho, const int N) : eomg(eomg_), ev(ev_)
	{
		Slist = S;
		M     = M_;

		N_steps = N + 1;

		// std::vector for SE(3) cartesian poses, desired trajectory
		FK_current.resize(N_steps);
		FK_current_pos.resize(3, N_steps);

		// joint space variables q, q_dot
		thetalist  = Eigen::MatrixXd::Zero(NDOF, N_steps);
	    thetalistd = Eigen::MatrixXd::Zero(NDOF, N_steps);

	    IK = IK_solver(Slist,  M, joint_limits, eomg, ev, rho);

	    // termination condition
	   	is_terminate = this->terminate;

	}

	/* inputs - poses_des, outputs - joint_positions */
	void getTrajectory(const std::vector<Eigen::MatrixXd>& FK_desired, const Eigen::VectorXd& q0, const Eigen::VectorXd& qd0,
	 const Eigen::MatrixXd& q_bar, const Eigen::MatrixXd& qd_bar, const Eigen::VectorXd& rho,  Eigen::MatrixXd* joint_positions) {

	    // thetalist.col(0)  = q0;
	    // thetalistd.col(0) = qd0;

		joint_positions->col(0) = q0;

	   	Eigen::VectorXd thetalist0  = q0;
	    Eigen::VectorXd thetalistd0 = qd0;

	    Eigen::VectorXd thetalist_ret(NDOF);
	    thetalist_ret = q0;

	    FK_current.at(0) = mr::FKinSpace(M, Slist, q0);
	    
	    bool initial = false;
	    double cost = 10;

	    int j = 0;
	    while (is_terminate(cost, j) == 0) {
	        for (int i = 1; i < N_steps; i++) {
	            
	        	if (j > 0) {
		            thetalist0   = joint_positions->col(i);
		            thetalistd0  = thetalistd.col(i);
	        	} else {
		            thetalist0   = thetalist_ret;
		            thetalistd0  = thetalistd.col(i);
	        	}

	            if (i == 1) {
	                thetalist0   = q0;
	                thetalistd0  = thetalistd.col(0);
	        	}

	            /* solves IK for each time step */
	            IK.getIK(FK_desired.at(i), thetalist0, thetalistd0, q_bar.col(i), qd_bar.col(i), initial, rho, &thetalist_ret);

	            joint_positions->col(i) = thetalist_ret;
	            FK_current.at(i) = mr::FKinSpace(M, Slist, joint_positions->col(i));
	            FK_current_pos.col(i) = FK_current.at(i).block(0, 3, 3, 1);

	        }  

	        cost = trajectoryCost(FK_desired);
	        j++;
	    }
	}

	/* Generates lissajous trajectories 
	takes a fixed orientation. TODO: make a variable of the surface normal
	*/
	std::vector<Eigen::MatrixXd> generateLissajousTrajectories(const Eigen::MatrixXd& R, double z, int n, int m, double r1, double r2, int N, double Tf) {
		double timegap = Tf / (N);
		std::vector<Eigen::MatrixXd> traj(N + 1);
		Eigen::Vector3d p;
		Eigen::MatrixXd T(4,4);
		double t = 0.0;

		for (int i = 0;i < N+1; i++) {
			p(0) = r1 * std::cos(n * t); p(1) = r2 * std::sin(m * t); p(2) = z;
			T = mr::RpToTrans(R, p);
			traj.at(i) = T;
			t = t + timegap;
		}
		return traj;

	}



	// returns current FK.
	inline Eigen::MatrixXd getFKCurrentPos() {
		return FK_current_pos;
	}

	/* Terminate condition */
	static inline int terminate(double cost, int iter) {
		if (cost < 0.01 || iter > 2) {
			return 1;
		} 
		return 0;
	}

private:
	/* return the current fk cost */
	double trajectoryCost(const std::vector<Eigen::MatrixXd>& FK_desired) {
		double cost = 0;
		for (int i = 0; i < N_steps; i++) {
			// std::cout << FK_current.at(i) << std::endl;
			cost = cost + (FK_desired.at(i) - FK_current.at(i)).norm();
		}
		return cost;
	}

public:
	double eomg;
	double ev;
	Eigen::MatrixXd Slist;
	Eigen::MatrixXd M;

	Eigen::MatrixXd thetalist;
	Eigen::MatrixXd thetalistd;

	std::vector<Eigen::MatrixXd> FK_current;
	Eigen::MatrixXd FK_current_pos;

	std::function<int(double, int)> is_terminate;

	IK_solver IK;
	int N_steps;

};


#endif // IK_TRAJECTORY_HPP