#include <memory>
#include <Eigen/Dense>

#include "config.h"
#include "ADMMTrajOptimizerMPC.hpp"
#include "RobotPlant.hpp"
#include "RobotDynamics.hpp"
#include "RobotPublisherMPC.hpp"



int main(int argc, char *argv[]) {

  /* -------------------- orocos kdl robot initialization-------------------------*/
  KUKAModelKDLInternalData robotParams;
  robotParams.numJoints = NDOF;
  robotParams.Kv = Eigen::MatrixXd(7,7);
  robotParams.Kp = Eigen::MatrixXd(7,7);

  // ---------------------------------- Define the robot and contact model ---------------------------------- 
  KDL::KukaDHKdl robot = KDL::KukaDHKdl();
  std::shared_ptr<RobotAbstract> kukaRobot = std::shared_ptr<RobotAbstract>(new KUKAModelKDL(robot(), robotParams));
  kukaRobot->initRobot();

  optimizer::IterativeLinearQuadraticRegulatorADMM::traj result;
  stateVec_t xinit;
  xinit.setZero();

  int ADMMiterMax = 3;
  ADMMopt ADMM_OPTS(TimeStep, 1e-7, 1e-7, 15, ADMMiterMax);



  Eigen::MatrixXd joint_lims(2,7);

  double eomg = 0.00001;
  double ev   = 0.00001;


  /* Cartesian Tracking. IKopt */
  IKTrajectory<IK_FIRST_ORDER>::IKopt IK_OPT(NDOF);
  models::KUKA robotIK = models::KUKA();
  Eigen::MatrixXd Slist(6, NDOF);
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
  double z_depth = 1.17;
  double r       = 0.05;
  std::vector<Eigen::MatrixXd> cartesianPoses = IK_traj.generateLissajousTrajectories(R, z_depth, 1, 1, r, r, NumberofKnotPt, Tf);


  // contact model parameters
  ContactModel::ContactParams<double> cp;
  cp.E = 1000;
  cp.mu = 0.5;
  cp.nu = 0.55;
  cp.R  = 0.005;
  cp.R_path = 1000;
  cp.Kd = 10;
  ContactModel::SoftContactModel<double> contactModel(cp);

  /*-----------------------------------------------------------------------------------------------------------------------*/
  double state_var   = 0.001;
  double control_var = 0.0001;

  double dt      = TimeStep;
  unsigned int horizon_mpc = 100;          // make these loadable from a cfg file


  std::shared_ptr<RobotAbstract> kukaRobot_plant = std::shared_ptr<RobotAbstract>(new KUKAModelKDL(robot(), robotParams));
  kukaRobot_plant->initRobot();
  ContactModel::SoftContactModel<double> contactModel_plant;

  contactModel_plant = contactModel;
  // Initialize Robot Model
  std::shared_ptr<RobotDynamics> KukaModel_plant{new RobotDynamics(dt, horizon_mpc, kukaRobot_plant, contactModel_plant)};

  using Plant_ = RobotPlant<RobotDynamics, stateSize, commandSize>;
  std::shared_ptr<Plant_> plant{new Plant_(KukaModel_plant, dt, state_var, control_var)};
  /*-----------------------------------------------------------------------------------------------------------------------*/

  // Initialize robot publisher
  using RobotPublisher = RobotPublisherMPC<Plant<stateSize, commandSize>, stateSize, commandSize>;
  std::shared_ptr<RobotPublisher> plantPublisher = std::shared_ptr<RobotPublisher>(new RobotPublisher(plant, static_cast<int>(horizon_mpc), static_cast<int>(NumberofKnotPt), dt));


  Saturation LIMITS;
  Eigen::VectorXd x_limits_lower(stateSize);
  Eigen::VectorXd x_limits_upper(stateSize);
  Eigen::VectorXd u_limits_lower(commandSize);
  Eigen::VectorXd u_limits_upper(commandSize);
  x_limits_lower << -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -10, -10, -10;    
  x_limits_upper << M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 10, 10, 10;      
  u_limits_lower << -20, -20, -20, -20, -20, -20, -20;
  u_limits_upper << 20, 20, 20, 20, 20, 20, 20;

  LIMITS.stateLimits.row(0)   = x_limits_lower;
  LIMITS.stateLimits.row(1)   = x_limits_upper;
  LIMITS.controlLimits.row(0) = u_limits_lower; 
  LIMITS.controlLimits.row(1) = u_limits_upper; 


  xinit.head(7) << 0, 0.2, 0, 0.5, 0, 0.2, 0;


  /* ------------------------ admm mpc ----------------------------- */
  ADMMTrajOptimizerMPC optimizerADMM;
  // admm_mpc(kukaRobot, plant, contactModel, xinit, cartesianPoses, result);
  optimizerADMM.run(kukaRobot, plantPublisher, xinit, contactModel, ADMM_OPTS, IK_OPT, LIMITS, cartesianPoses, result);
  /* ------------------------ admm mpc ----------------------------- */

  // admm_mpc(kukaRobot, xinit, cartesianPoses, result);

  return 0;

}