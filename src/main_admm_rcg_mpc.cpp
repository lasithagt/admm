#include <memory>
#include <Eigen/Dense>

#include "config.h"
#include "ADMMTrajOptimizerMPC.hpp"
#include "robot_plant.hpp"
#include "robot_dynamics.hpp"
#include "RobCodGenModel.h"
#include "admm_public.hpp"
#include "utils.h"

int main(int argc, char *argv[]) {


  // Define the robot and contact model
  std::shared_ptr<RobotAbstract> kukaRobot = std::shared_ptr<RobotAbstract>(new RobCodGenModel());
  kukaRobot->initRobot();

  optimizer::IterativeLinearQuadraticRegulatorADMM::traj result;
  stateVec_t xinit;
  xinit.setZero();

  stateVecTab_t xtrack;
  xtrack.resize(stateSize, NumberofKnotPt + 1);

  int ADMMiterMax = 6;
  unsigned int ddpIter = 10;
  ADMMopt ADMM_OPTS(TimeStep, 1e-7, 1e-7, ddpIter, ADMMiterMax);


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
  IK_OPT.ev           = ev;
  IK_OPT.eomg         = eomg;
  IK_OPT.Slist        = Slist;
  IK_OPT.M            = M;


  // desired tracking trajectory
  TrajectoryDesired<stateSize, NumberofKnotPt> desiredTrajectory;


  Eigen::MatrixXd R(3,3);
  R << 1, 0, 0, 
       0, 1, 0, 
       0, 0, 1;
  double Tf      = 2 * M_PI;
  double z_depth = 1.17;
  double r       = 0.05;

  std::vector<Eigen::MatrixXd> cartesianPoses = admm::utils::generateLissajousTrajectories(R, z_depth, 1, 1, r, r, NumberofKnotPt, Tf);

  desiredTrajectory.cartesianTrajectory     = cartesianPoses;
  desiredTrajectory.stateTrajectory.row(16) = 0 * Eigen::VectorXd::Ones(NumberofKnotPt + 1);



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
  double state_var         = 0.000001;
  double control_var       = 0.0001;

  double dt                = TimeStep;
  unsigned int horizon_mpc = 100;          // make these loadable from a cfg file


  // state and control limits
  Saturation LIMITS;

  LIMITS.stateLimits.row(0)   << -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -2.0, -10, -10, -10;
  LIMITS.stateLimits.row(1)   << M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 10, 10, 10;
  LIMITS.controlLimits.row(0) << -20, -20, -20, -20, -20, -20, -20; 
  LIMITS.controlLimits.row(1) << 20, 20, 20, 20, 20, 20, 20; 


  ADMM_MPCopt ADMM_MPC_OPT       = ADMM_MPCopt(ADMM_OPTS, LIMITS);
  ADMM_MPCconfig ADMM_MPC_CONFIG = ADMM_MPCconfig(ADMM_MPC_OPT, IK_OPT, dt, horizon_mpc);


  std::shared_ptr<RobotAbstract> kukaRobot_plant = std::shared_ptr<RobotAbstract>(new RobCodGenModel());
  kukaRobot_plant->initRobot();
  ContactModel::SoftContactModel<double> contactModel_plant;

  contactModel_plant = contactModel;

  // Initialize the Robot Dynamical Model
  using Dynamics = admm::Dynamics<RobotAbstract, stateSize, commandSize>;
  std::shared_ptr<Dynamics> KukaModel_plant{new RobotDynamics(dt, horizon_mpc, kukaRobot_plant, contactModel_plant)};
  std::shared_ptr<Plant<Dynamics, stateSize, commandSize>> plant{new RobotPlant<Dynamics, stateSize, commandSize>(KukaModel_plant, dt, state_var, control_var)};


  // Initialize robot publisher
  using RobotPublisher = RobotPublisherMPC<Plant<Dynamics, stateSize, commandSize>, stateSize, commandSize>;
  std::shared_ptr<RobotPublisher> plantPublisher = std::shared_ptr<RobotPublisher>(new RobotPublisher(plant, static_cast<int>(horizon_mpc), static_cast<int>(NumberofKnotPt), dt));


  Eigen::VectorXd q_bar(7);
  Eigen::VectorXd thetalist_ret(7);

  q_bar << 0, 0, 0, 0, 0, 0, 0;

  Eigen::VectorXd rho_init(5);
  rho_init << 0, 0, 0, 0, 0;
  IK_FIRST_ORDER IK = IK_FIRST_ORDER(IK_OPT.Slist,  IK_OPT.M, IK_OPT.joint_limits, IK_OPT.eomg, IK_OPT.ev, rho_init);

  IK.getIK_random_initial(desiredTrajectory.cartesianTrajectory.at(0), q_bar,rho_init, thetalist_ret);

  xinit.head(7) = thetalist_ret;
  plant->setInitialState(xinit); 

  ADMMTrajOptimizerMPC<RobotAbstract, RobotAbstract, stateSize, commandSize> optimizerADMM;
  optimizerADMM.run(kukaRobot, plantPublisher, xinit, contactModel, ADMM_MPC_CONFIG, desiredTrajectory, result);

  return 0;

}