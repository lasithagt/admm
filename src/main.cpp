#include <memory>
#include <Eigen/Dense>


#include "admm_main.hpp"
#include "config.h"
// #include "admm_mpc_main.hpp"

void generateCartesianTrajectory(stateVec_t& xinit, stateVec_t& xgoal, stateVecTab_t& xtrack, std::vector<Eigen::MatrixXd> &cartesianPoses);
void admm_mpc(std::shared_ptr<RobotAbstract>& kukaRobot, stateVec_t init_state, optimizer::ILQRSolverADMM::traj& result);
void admm(std::shared_ptr<RobotAbstract>& kukaRobot, stateVec_t init_state, optimizer::ILQRSolverADMM::traj& result);



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

  xinit.head(7) << 0.1, 0.2, 0.1, 0.2, 0.1, 0.1, 0.1;

  admm(kukaRobot, xinit, result);

}