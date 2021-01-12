#include <memory>
#include <Eigen/Dense>


#include "ADMMTrajOptimizer.hpp"
#include "ADMMTrajOptimizerMPC.hpp"

#include "config.h"


/* -------------------------------------------------------------- admm mpc solver -------------------------------------------------------- */ 
void admm_mpc(std::shared_ptr<RobotAbstract>& kukaRobot, stateVec_t init_state, std::vector<Eigen::MatrixXd>& cartesianPoses, optimizer::IterativeLinearQuadraticRegulatorADMM::traj& result)
{

  ADMMTrajOptimizerMPC optimizerADMM;
  stateVecTab_t xtrack;
  xtrack.resize(stateSize, NumberofKnotPt + 1);

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


  // contact model dynamics
  ContactModel::ContactParams<double> cp_;
  cp_.E = 1000;
  cp_.mu = 0.5;
  cp_.nu = 0.55;
  cp_.R  = 0.005;
  cp_.R_path = 1000;
  cp_.Kd = 10;
  ContactModel::SoftContactModel<double> contactModel(cp_);
  kukaRobot->initRobot();


  stateVec_t xinit;
  xinit.setZero();
  xinit.head(7) << 0, 0.2, 0, 0.5, 0, 0.2, 0;

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


  optimizerADMM.run(kukaRobot, xinit, contactModel, ADMM_OPTS, IK_OPT, LIMITS, cartesianPoses, result);

}

/* -------------------------------------------  run admm  full trajectory optimization ------------------------------------------------ */
void admm(std::shared_ptr<RobotAbstract>& kukaRobot, stateVec_t init_state, std::vector<Eigen::MatrixXd>& cartesianPoses, optimizer::IterativeLinearQuadraticRegulatorADMM::traj& result)
{

  unsigned int N = NumberofKnotPt;
  int ADMMiterMax = 5;
  double dt = TimeStep;

  ADMMopt ADMM_OPTS(dt, 1e-7, 1e-7, 15, ADMMiterMax);

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

  unsigned int iterMax = 10; // DDP iteration max

  /* -------------------- Optimizer Params ------------------------ */
  optimizer::IterativeLinearQuadraticRegulatorADMM::OptSet solverOptions;
  solverOptions.n_hor    = N;
  solverOptions.tolFun   = ADMM_OPTS.tolFun;
  solverOptions.tolGrad  = ADMM_OPTS.tolGrad;
  solverOptions.max_iter = iterMax;


  ContactModel::ContactParams<double> cp_;
  cp_.E = 1000;
  cp_.mu = 0.5;
  cp_.nu = 0.4;
  cp_.R  = 0.005;
  cp_.R_path = 1000;
  cp_.Kd = 10;
  ContactModel::SoftContactModel<double> contactModel(cp_);
  kukaRobot->initRobot();

  /* ---------------------------------------------------------------------------------------------- */

  /* ---------------------------------- State and Control Limits ---------------------------------- */
  Saturation LIMITS;
  Eigen::VectorXd x_limits_lower(stateSize);
  Eigen::VectorXd x_limits_upper(stateSize);
  Eigen::VectorXd u_limits_lower(commandSize);
  Eigen::VectorXd u_limits_upper(commandSize);
  x_limits_lower << -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -10, -10, -10;    
  x_limits_upper << M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 10, 10, 10;      
  u_limits_lower << -20, -20, -20, -20, -20, -20, -20;
  u_limits_upper << 20, 20, 20, 20, 20, 20, 20;

  LIMITS.stateLimits.row(0) = x_limits_lower;
  LIMITS.stateLimits.row(1) = x_limits_upper;
  LIMITS.controlLimits.row(0) = u_limits_lower; 
  LIMITS.controlLimits.row(1) = u_limits_upper; 

  /* ----------------------------------------------------------------------------------------------- */



  // admm optimizer
  ADMMTrajOptimizer admm_full = ADMMTrajOptimizer(N, TimeStep);

  admm_full.run(kukaRobot, init_state, solverOptions, ADMM_OPTS, IK_OPT, LIMITS, cp_, cartesianPoses);


  // get the final trajectory
  result = admm_full.getOptimizerResult();



}
