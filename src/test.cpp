#include "SoftContactModel.h"
#include "kuka_arm.h"
#include "SoftContactModel.h"
#include "KukaModel.h"
#include "cost_function_admm.h"
#include "models.h"
#include "config.h"

// Test scripts
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");


int main() {

	KUKAModelKDLInternalData robotParams;
	robotParams.numJoints = 7;
	robotParams.Kv = Eigen::MatrixXd(7,7);
	robotParams.Kp = Eigen::MatrixXd(7,7);

	/* ---------------------------------- Define the robot and contact model ---------------------------------- */
	KDL::Chain robot = KDL::KukaDHKdl();
	std::shared_ptr<RobotAbstract> kukaRobot = std::shared_ptr<RobotAbstract>(new KUKAModelKDL(robot, robotParams));

	ContactModel::ContactParams cp_;
	cp_.E = 300;
	cp_.mu = 0.4340;
	cp_.nu = 0.55;
	cp_.R  = 0.0013/2;
	cp_.R_path = 1000;
	cp_.Kd = 100;

	ContactModel::SoftContactModel contactModel(cp_);
	kukaRobot->initRobot();

	// // ----------------------------------------------------------------------------------------------------------
	/* TEST Soft Contact Model */
	Eigen::Matrix3d mass_matrix_cart;
	mass_matrix_cart << 1, 0, 0, 0, 1, 0, 0, 0, 1;

	Eigen::Vector3d position;
	position << 0,0,0;

	Eigen::Vector3d velocity;
	velocity << 0,0,0.01;

	Eigen::Vector3d acceleration;
	acceleration << 0,0,0;	

	Eigen::Vector3d force_current;
	force_current << 0, 0, 1;

	Eigen::Vector3d force_next;

	// contactModel.df(mass_matrix_cart, position, position, velocity, acceleration, force_current, force_next);
	// std::cout << force_next.transpose().format(CleanFmt) << std::endl;

	// ----------------------------------------------------------------------------------------------------------
	// TEST manipulator + contact dynamics model
	double dt = 0.02;
	unsigned int D = 7;
	KukaArm plant = KukaArm(dt, D, kukaRobot, contactModel); 
	stateVec_t x;
	commandVec_t u;
	u << 0, 0, 0, 0, 0, 0, 0;
	x << M_PI/3, M_PI/3, M_PI/3, M_PI/3, M_PI/3, M_PI/3, M_PI/3, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0, 0, 1; 
	stateVec_t dyn = plant.kuka_arm_dynamics(x, u);
	std::cout << "\n" << dyn.transpose().format(CleanFmt) << "\n" << std::endl;
	// ----------------------------------------------------------------------------------------------------------

	// // ----------------------------------------------------------------------------------------------------------
	/* TEST cost function */
	stateVec_t x_goal(stateSize);
	x_goal << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	stateVecTab_t x_track(stateSize, 1);
	x_track.col(0) = x_goal;

	// CostFunctionADMM costFunction = CostFunctionADMM(x_goal, x_track);

	// std::cout << costFunction.cost_func_expre(0, x, u) << std::endl;

	// stateVecTab_t xList;
	// xList.resize(stateSize, 1);
	// xList.col(0) = x;

	// commandVecTab_t uList;
	// uList.resize(commandSize, 1);
	// uList.col(0) = u;

	// Eigen::MatrixXd cList_bar;
	// cList_bar.resize(2,1);

	// stateVecTab_t xList_bar;
	// xList_bar.resize(stateSize, 1);

	// commandVecTab_t uList_bar;
	// uList_bar.resize(commandSize, 1);

	// Eigen::MatrixXd thetaList_bar;
	// thetaList_bar.resize(7,1);

	// Eigen::VectorXd rho(5);
	// rho << 1,2,3,0,4;


	// costFunction.computeDerivatives(xList, uList, cList_bar, xList_bar, uList_bar, thetaList_bar, rho);


	// // ----------------------------------------------------------------------------------------------------------




}


