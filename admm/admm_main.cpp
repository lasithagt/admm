
#include "admm_main.hpp"

/* ------------- Eigen print arguments ------------------- */
  // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
 /* ------------------------------------------------------- */


FULL_ADMM::FULL_ADMM(unsigned int N_, double dt_) : N(N_), dt(dt_) {
}

FULL_ADMM::~FULL_ADMM() {}

void FULL_ADMM::run(std::shared_ptr<RobotAbstract>& kukaRobot, stateVec_t init_state, optimizer::ILQRSolverADMM::OptSet& solverOptions, ADMM::ADMMopt& ADMM_OPTS, IKTrajectory<IK_FIRST_ORDER>::IKopt& IK_OPT) 
{
  
  // parameters for ADMM, penelty terms. initial
  Eigen::VectorXd rho_init(5);
  rho_init << 0, 0, 0, 0, 0;


  IKTrajectory<IK_FIRST_ORDER> IK_traj = IKTrajectory<IK_FIRST_ORDER>(IK_OPT.Slist, IK_OPT.M, IK_OPT.joint_limits, IK_OPT.eomg, IK_OPT.ev, rho_init, N);
  // cost function. TODO: make this updatable
  CostFunctionADMM costFunction_admm(N, kukaRobot);

  ContactModel::ContactParams cp_;
  cp_.E = 4000;
  cp_.mu = 0.5;
  cp_.nu = 0.55;
  cp_.R  = 0.005;
  cp_.R_path = 1000;
  cp_.Kd = 10;
  ContactModel::SoftContactModel contactModel(cp_);
  kukaRobot->initRobot();


  // dynamic model of the manipulator and the contact model
  KukaArm KukaArmModel(dt, N, kukaRobot, contactModel);


  // TODO: make this updatable, for speed
  optimizer::ILQRSolverADMM solverDDP(KukaArmModel, costFunction_admm, solverOptions, N, ADMM_OPTS.dt, ENABLE_FULLDDP, ENABLE_QPBOX);


  // admm optimizer
  ADMM optimizerADMM(kukaRobot, costFunction_admm, solverDDP, ADMM_OPTS, IK_OPT, N);


  stateVec_t xinit, xgoal;
  stateVecTab_t xtrack;
  xtrack.resize(stateSize, NumberofKnotPt + 1);
  xtrack.row(16) = 2 * Eigen::VectorXd::Ones(NumberofKnotPt + 1); 


  /* ------------------------------------------------------------------------------------------------------ */

  /* ---------------------------------- State and Control Limits ---------------------------------- */
  ADMM::Saturation LIMITS;
  Eigen::VectorXd x_limits_lower(stateSize);
  Eigen::VectorXd x_limits_upper(stateSize);
  Eigen::VectorXd u_limits_lower(commandSize);
  Eigen::VectorXd u_limits_upper(commandSize);
  x_limits_lower << -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -0.10, -0.10, -0.10, -0.10, -0.10, -0.10, -0.10, -10, -10, -10;    
  x_limits_upper << M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 10, 10, 10;      
  u_limits_lower << -1, -1, -1, -1, -1, -1, -1;
  u_limits_upper << 1, 1, 1, 1, 1, 1, 1;

  LIMITS.stateLimits.row(0) = x_limits_lower;
  LIMITS.stateLimits.row(1) = x_limits_upper;
  LIMITS.controlLimits.row(0) = u_limits_lower; 
  LIMITS.controlLimits.row(1) = u_limits_upper; 

  /* ----------------------------------------------------------------------------------------------- */

  /* State Tracking. Force tracking */
  Eigen::MatrixXd F(3, N + 1);
  F.setZero();
  // F.row
  // xtrack.block(14, 0, 3, xtrack.cols()) = F;

  Eigen::MatrixXd R(3,3);
  R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  double Tf = 2 * M_PI;


  std::vector<Eigen::MatrixXd> cartesianPoses = IK_traj.generateLissajousTrajectories(R, 1.16, 1, 3, 0.05, 0.05, N, Tf);

  /* initialize xinit, xgoal, xtrack - for the hozizon*/
  Eigen::VectorXd thetalist0(7);
  Eigen::VectorXd thetalistd0(7);
  Eigen::VectorXd q_bar(7);
  Eigen::VectorXd qd_bar(7);
  Eigen::VectorXd thetalist_ret(7);
  // thetalist0 << 0, 0.2, 0, 0.5, 0, 0.2, 0;
  for (int i = 0;i < 7; i++) { thetalist0(i) = init_state(i);}

  thetalistd0 << 0, 0, 0, 0, 0, 0, 0;
  q_bar << 0, 0, 0, 0, 0, 0, 0;
  qd_bar << 0, 0, 0, 0, 0, 0, 0;

  bool initial = true;
  IK_FIRST_ORDER IK = IK_FIRST_ORDER(IK_OPT.Slist,  IK_OPT.M, IK_OPT.joint_limits, IK_OPT.eomg, IK_OPT.ev, rho_init);

  IK.getIK(cartesianPoses.at(0), thetalist0, thetalistd0, q_bar, qd_bar, initial, rho_init, &thetalist_ret);
  xinit.head(7) = thetalist_ret;

  std::cout << thetalist_ret << std::endl;

  /* ----------------------------------------------------------------------------------------------------------------------------------*/
  Eigen::Vector3d vel;
  Eigen::Vector3d accel;
  Eigen::Vector3d poseP;
  Eigen::Matrix<double, 3, 3> poseM;
  kukaRobot->getForwardKinematics(thetalist_ret.data(), thetalist_ret.data(), thetalist_ret.data(), poseM, poseP, vel, accel, false);

  std::cout << cartesianPoses.at(0) << std::endl;
  std::cout << poseM << std::endl;
  std::cout << poseP << std::endl;

  // thetalist_ret <<  -0.5, -0.5, 0.8, 0.1, 0.2, 0.5, 0.5;
  // std::cout << IK_OPT.M << std::endl;
  // std::cout << IK_OPT.Slist << std::endl;
  std::cout << mr::FKinSpace(IK_OPT.M, IK_OPT.Slist, thetalist_ret) << std::endl;
  /* ----------------------------------------------------------------------------------------------------------------------------------*/

  // xinit = init_state;

  Eigen::VectorXd rho(5);
  rho << 1, 0.1, 0.05, 0, 1;

  commandVecTab_t u_0;
  u_0.resize(commandSize, N);
  u_0.setZero();

  optimizerADMM.solve(xinit, u_0, xtrack, cartesianPoses, rho, LIMITS);


  resultTrajectory = optimizerADMM.getLastSolvedTrajectory();

}

optimizer::ILQRSolverADMM::traj FULL_ADMM::getOptimizerResult() 
{
  return resultTrajectory;
}
