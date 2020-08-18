#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "modern_robotics.h"
#include "ik_trajectory.hpp"
#include "ik_solver.hpp"
#include "kuka_robot.hpp"
#include <chrono>
#include <ctime>
#include <math.h> 
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> newType;


int main() {

	// random testing
	newType a(10);


	/* Test the manipulator dynamics.

	*/
	models::KUKA robot = models::KUKA();
	Eigen::MatrixXd Slist(6,7);
	Eigen::MatrixXd M(4,4);

	robot.getSlist(&Slist); 
	robot.getM(&M);


	// Test IK_FIRST_ORDER
	Eigen::MatrixXd joint_limits(2,7);
	double eomg = 0.00001;
	double ev   = 0.00001;
	Eigen::VectorXd rho(3);
	rho << 0, 0, 0;
	Eigen::MatrixXd Td(4,4);
	Eigen::VectorXd thetalist0(7);
	Eigen::VectorXd thetalistd0(7);
	Eigen::VectorXd q_bar(7);
	Eigen::VectorXd qd_bar(7);
	Eigen::VectorXd thetalist_ret(7);
	Td << 1, 0, 0, 0.08, 0, 1, 0, 0.1, 0, 0, 1, 0.8, 0, 0, 0, 1;
	thetalist0 << 0.1, 0.2, 0.1, 0.2, 0.1, 0.1, 0.1;
	thetalistd0 << 0, 0, 0, 0, 0, 0, 0;
	q_bar << 0, 0, 0, 0, 0, 0, 0;
	qd_bar << 0, 0, 0, 0, 0, 0, 0;



	/* IK Example 
	Td          = [1 0 0 0.08;0 1 0 0;0 0 1 0.8;0 0 0 1]
	thetalist0  = [0.1, 0.2, 0.1, 0.2, 0.1, 0.1, 0.1]
	thetalistd0 = [0, 0, 0, 0, 0, 0, 0]
	q_bar       = [0, 0, 0, 0, 0, 0, 0] 
	qd_bar 		= [0, 0, 0, 0, 0, 0, 0]
	*/
	using namespace std::chrono;

	bool initial = true;
	IK_FIRST_ORDER IK = IK_FIRST_ORDER(Slist,  M, joint_limits, eomg, ev, rho);

	high_resolution_clock::time_point t1 = high_resolution_clock::now();
	IK.getIK(Td, thetalist0, thetalistd0, q_bar, qd_bar, initial, rho, &thetalist_ret);
	high_resolution_clock::time_point t2 = high_resolution_clock::now();

	duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

	std::cout << "It took me " << time_span.count() << " seconds.";
	std::cout << std::endl;

	std::cout << Td << std::endl;
	std::cout << mr::FKinSpace(M, Slist, thetalist_ret)  << std::endl;


	int N = 10;
	std::vector<Eigen::MatrixXd> FK_poses;
	FK_poses.push_back(Td);
	Eigen::MatrixXd joint_positions(7, N);
	Eigen::MatrixXd q_barM(7, N);
	Eigen::MatrixXd qd_barM(7, N);
	joint_positions.col(0) = thetalist_ret;

	IKTrajectory<IK_FIRST_ORDER> IK_traj = IKTrajectory<IK_FIRST_ORDER>(Slist, M, joint_limits, eomg, ev, rho, N);


	/* Generates a cartesian trajectory
	Given lissajous parameters, returns a std::vector of the poses. Example, 
	*/
	Eigen::MatrixXd R(3,3);
	R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
	double Tf = 2 * M_PI;
	std::vector<Eigen::MatrixXd> poses = IK_traj.generateLissajousTrajectories(R, 0.8, 1, 3, 0.08, 0.08, N, Tf);
	poses.at(0) = Td;

	// IK trajectory
	t1 = high_resolution_clock::now();
	IK_traj.getTrajectory(poses, thetalist_ret, thetalistd0, q_barM, qd_barM, rho,  &joint_positions);
	t2 = high_resolution_clock::now();
	time_span = duration_cast<duration<double>>(t2 - t1);
	std::cout << "It took me " << time_span.count() << " seconds.";
	std::cout << std::endl;

	std::cout << joint_positions.col(5) << std::endl;

	return 0;
}