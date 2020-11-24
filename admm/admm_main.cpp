
#include "admm_main.hpp"

/* ------------- Eigen print arguments ------------------- */
  // Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
 /* ------------------------------------------------------- */


FULL_ADMM::FULL_ADMM(unsigned int N_, double dt_) : N(N_), dt(dt_) {
}

FULL_ADMM::~FULL_ADMM() {}

void FULL_ADMM::run(std::shared_ptr<RobotAbstract>& kukaRobot, stateVec_t init_state, optimizer::ILQRSolverADMM::OptSet& solverOptions, ADMM::ADMMopt& ADMM_OPTS, IKTrajectory<IK_FIRST_ORDER>::IKopt& IK_OPT, ADMM::Saturation& LIMITS, ContactModel::ContactParams& cp, std::vector<Eigen::MatrixXd>& cartesianPoses) 

{
  
  // parameters for ADMM, penelty terms. initial
  Eigen::VectorXd rho_init(5);
  rho_init << 0, 0, 0, 0, 0;

  // cost function. TODO: make this updatable
  CostFunctionADMM costFunction_admm(N, kukaRobot);


  ContactModel::SoftContactModel contactModel(cp);
  kukaRobot->initRobot();


  // dynamic model of the manipulator and the contact model
  KukaArm KukaArmModel(dt, N, kukaRobot, contactModel);


  // TODO: make this updatable, for speed
  optimizer::ILQRSolverADMM solverDDP(KukaArmModel, costFunction_admm, solverOptions, N, ADMM_OPTS.dt, ENABLE_FULLDDP, ENABLE_QPBOX);


  // admm optimizer
  ADMM optimizerADMM(kukaRobot, costFunction_admm, solverDDP, ADMM_OPTS, IK_OPT, N);


  stateVec_t xinit;
  stateVecTab_t xtrack;
  xtrack.resize(stateSize, NumberofKnotPt + 1);
  xtrack.row(16) = 2 * Eigen::VectorXd::Ones(NumberofKnotPt + 1); 


  /* initialize xinit, xgoal, xtrack - for the hozizon*/
  Eigen::VectorXd thetalist0(7);
  Eigen::VectorXd thetalistd0(7);
  Eigen::VectorXd q_bar(7);
  Eigen::VectorXd qd_bar(7);
  Eigen::VectorXd thetalist_ret(7);


  for (int i = 0;i < 7; i++) { thetalist0(i) = init_state(i);}

  thetalistd0 << 0, 0, 0, 0, 0, 0, 0;
  q_bar << 0, 0, 0, 0, 0, 0, 0;
  qd_bar << 0, 0, 0, 0, 0, 0, 0;

  bool initial = true;
  IK_FIRST_ORDER IK = IK_FIRST_ORDER(IK_OPT.Slist,  IK_OPT.M, IK_OPT.joint_limits, IK_OPT.eomg, IK_OPT.ev, rho_init);

  IK.getIK(cartesianPoses.at(0), thetalist0, thetalistd0, q_bar, qd_bar, initial, rho_init, thetalist_ret);
  xinit.head(7) = thetalist_ret;

  // xinit = init_state;

  Eigen::VectorXd rho(5);
  rho << 6, 0.1, 0.001, 0, 2;

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
