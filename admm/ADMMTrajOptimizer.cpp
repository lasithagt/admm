#include "ADMMTrajOptimizer.hpp"
#include "RobotDynamics.hpp"
#include "eigenmvn.hpp"
#include "cnpy.h"


ADMMTrajOptimizer::ADMMTrajOptimizer(unsigned int N_, double dt_) : N(N_), dt(dt_) {}

ADMMTrajOptimizer::~ADMMTrajOptimizer() {}

void ADMMTrajOptimizer::run(const std::shared_ptr<RobotAbstract>& kukaRobot, stateVec_t init_state, optimizer::IterativeLinearQuadraticRegulatorADMM::OptSet& solverOptions, ADMMopt& ADMM_OPTS, IKTrajectory<IK_FIRST_ORDER>::IKopt& IK_OPT, \
 Saturation& LIMITS, ContactModel::ContactParams<double>& cp, std::vector<Eigen::MatrixXd>& cartesianPoses) 

{
  
  // parameters for ADMM, penelty terms. initial
  Eigen::VectorXd rho_init(5);
  rho_init << 0, 0, 0, 0, 0;

  // cost function. TODO: make this updatable
  std::shared_ptr<CostFunctionADMM> costFunction_admm = std::make_shared<CostFunctionADMM>(N, kukaRobot);


  ContactModel::SoftContactModel<double> contactModel(cp);
  // kukaRobot->initRobot();


  // dynamic model of the manipulator and the contact model
  using Dynamics = admm::Dynamics<RobotAbstract, stateSize, commandSize>;
  std::shared_ptr<Dynamics> KukaDynModel = std::shared_ptr<Dynamics>(new RobotDynamics(dt, N, kukaRobot, contactModel));
  // std::shared_ptr<RobotDynamics> KukaDynModel = std::make_shared<RobotDynamics>(dt, N, kukaRobot, contactModel);

  // TODO: make this updatable, for speed
  using Optimizer = optimizer::IterativeLinearQuadraticRegulatorADMM;
  std::shared_ptr<Optimizer> solverDDP = std::make_shared<Optimizer>(KukaDynModel, costFunction_admm, solverOptions, N, ADMM_OPTS.dt, ENABLE_FULLDDP, ENABLE_QPBOX);


  // admm optimizer
  ADMMMultiBlock optimizerADMM(kukaRobot, costFunction_admm, solverDDP, ADMM_OPTS, IK_OPT, N);
  


  stateVec_t xinit;
  stateVecTab_t xtrack;
  xtrack.resize(stateSize, NumberofKnotPt + 1);
  xtrack.row(16) = 0 * Eigen::VectorXd::Ones(NumberofKnotPt + 1); 


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
  rho << 25, 0.1, 0.001, 0, 2;

  commandVecTab_t u_0;
  u_0.resize(commandSize, N);
  u_0.setZero();

  std::cout << "Running the Optimizer..." << std::endl;
  optimizerADMM.solve(xinit, u_0, xtrack, cartesianPoses, rho, LIMITS);


  resultTrajectory = optimizerADMM.getLastSolvedTrajectory();

  using State             = stateVec_t;
  using Scalar            = double;
  using Control           = commandVec_t;
  using StateNoiseVariance    = Eigen::Matrix<Scalar, stateSize, stateSize>;
  using ControlNoiseVariance  = Eigen::Matrix<Scalar, commandSize, commandSize>;

  Eigen::EigenMultivariateNormal<double, stateSize> sdist_(State::Zero(), 0.001 * StateNoiseVariance::Identity());
  Eigen::EigenMultivariateNormal<double, commandSize> cdist_(Control::Zero(), 0.001 * ControlNoiseVariance::Identity());




  /* simulate with disturbance on the open loop trjacetory */
  auto controlTrajectory = resultTrajectory.uList;
  auto stateTrajectory = resultTrajectory.xList;

  auto KList = resultTrajectory.KList;
  auto kList = resultTrajectory.kList;

  commandVec_t input;
  stateVec_t currentState;
  stateVecTab_t stateTrajectoryDisturbances(stateSize, N + 1);

  currentState = stateTrajectory.col(0);
  stateTrajectoryDisturbances.col(0) = currentState;

  // simulate trajectory with disurbances
  for (int i=0;i < N; i++)
  {
    input = controlTrajectory.col(i) + 1*cdist_.samples(1) - KList[i] * (stateTrajectory.col(i) - currentState);
    currentState += KukaDynModel->f(currentState, input) * TimeStep + 0.01*sdist_.samples(1);
    stateTrajectoryDisturbances.col(i+1) = currentState;
  }




  // Eigen::MatrixXd cartesian_mpc_disturbance_logger;
  // cartesian_mpc_disturbance_logger.resize(3, N + 1);

  // // save data
  // for (int i = 0; i < N+1; i++) 
  // {
  //   auto actual_cartesian_pose = mr::FKinSpace(IK_OPT.M, IK_OPT.Slist, stateTrajectoryDisturbances.col(i).head(7));
  //   cartesian_mpc_disturbance_logger.col(i) = actual_cartesian_pose.col(3).head(3);

  // }
  
  // cnpy::npy_save("./state_trajectory_admm_mpc_test.npy", cartesian_mpc_disturbance_logger.data(),
  //                 {1, static_cast<unsigned long>(cartesian_mpc_disturbance_logger.cols()),
  //                  static_cast<unsigned long>(cartesian_mpc_disturbance_logger.rows())}, "w");
    




}

optimizer::IterativeLinearQuadraticRegulatorADMM::traj ADMMTrajOptimizer::getOptimizerResult() 
{
  return resultTrajectory;
}
