#include <memory>
#include <Eigen/Dense>


#include "admm_main.hpp"
#include "config.h"
// #include "admm_mpc_main.hpp"

void generateCartesianTrajectory(stateVec_t& xinit, stateVec_t& xgoal, stateVecTab_t& xtrack, std::vector<Eigen::MatrixXd> &cartesianPoses);
void admm_mpc(std::shared_ptr<RobotAbstract>& kukaRobot, stateVec_t init_state, optimizer::ILQRSolverADMM::traj& result);
void admm(std::shared_ptr<RobotAbstract>& kukaRobot, stateVec_t init_state, std::vector<Eigen::MatrixXd>& cartesianPoses, optimizer::ILQRSolverADMM::traj& result);



int main(int argc, char *argv[]) {

  /* -------------------- orocos kdl robot initialization-------------------------*/
  KUKAModelKDLInternalData robotParams;
  robotParams.numJoints = NDOF;
  robotParams.Kv = Eigen::MatrixXd(7,7);
  robotParams.Kp = Eigen::MatrixXd(7,7);

  // ---------------------------------- Define the robot and contact model ---------------------------------- 
  KDL::Chain robot = KDL::KukaDHKdl();
  std::shared_ptr<RobotAbstract> kukaRobot = std::shared_ptr<RobotAbstract>(new KUKAModelKDL(robot, robotParams));

  optimizer::ILQRSolverADMM::traj result;
  stateVec_t xinit;
  xinit.setZero();

  xinit.head(7) << 0, 0.2, 0, 0.5, 0, 0.2, 0;

  Eigen::MatrixXd joint_lims(2,7);

  double eomg = 0.00001;
  double ev   = 0.00001;

  /* Cartesian Tracking. IKopt */
  IKTrajectory<IK_FIRST_ORDER>::IKopt IK_OPT(7);
  models::KUKA robotIK = models::KUKA();
  Eigen::MatrixXd Slist(6,7);
  Eigen::MatrixXd M(4,4);
  robotIK.getSlist(&Slist); 
  robotIK.getM(&M);

  IK_OPT.joint_limits = joint_lims;
  IK_OPT.ev = ev;
  IK_OPT.eomg = eomg;
  IK_OPT.Slist = Slist;
  IK_OPT.M = M;

  // parameters for ADMM, penelty terms. initial
  Eigen::VectorXd rho_init(5);
  rho_init << 0, 0, 0, 0, 0;
  IKTrajectory<IK_FIRST_ORDER> IK_traj = IKTrajectory<IK_FIRST_ORDER>(IK_OPT.Slist, IK_OPT.M, IK_OPT.joint_limits, IK_OPT.eomg, IK_OPT.ev, rho_init, NumberofKnotPt);

  Eigen::MatrixXd R(3,3);
  R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  double Tf = 2 * M_PI;
  // double z_depth = 1.161;
  double z_depth = 1.17;
  double r       = 0.05;
  std::vector<Eigen::MatrixXd> cartesianPoses = IK_traj.generateLissajousTrajectories(R, z_depth, 1, 3, r, r, NumberofKnotPt, Tf);



  admm(kukaRobot, xinit, cartesianPoses, result);

}