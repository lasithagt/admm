#include <memory>
#include <Eigen/Dense>


#include "admm.hpp"



/* ------------- Eigen print arguments ------------------- */
  Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
 /* ------------------------------------------------------- */


class FULL_ADMM {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FULL_ADMM(unsigned int N_, double dt_) : N(N_), dt(dt_) {

    }
    ~FULL_ADMM() {}

    void run(std::shared_ptr<RobotAbstract>& kukaRobot, optimizer::ILQRSolverADMM::OptSet& solverOptions, ADMM::ADMMopt& ADMM_OPTS, IKTrajectory<IK_FIRST_ORDER>::IKopt& IK_OPT) 
    {
      
      // parameters for ADMM, penelty terms. initial
      Eigen::VectorXd rho_init(5);
      rho_init << 0, 0, 0, 0, 0;
  

      IKTrajectory<IK_FIRST_ORDER> IK_traj = IKTrajectory<IK_FIRST_ORDER>(IK_OPT.Slist, IK_OPT.M, IK_OPT.joint_limits, IK_OPT.eomg, IK_OPT.ev, rho_init, N);
      // cost function. TODO: make this updatable
      CostFunctionADMM costFunction_admm(N, kukaRobot);

      ContactModel::ContactParams cp_;
      cp_.E = 1000;
      cp_.mu = 0.5;
      cp_.nu = 0.4;
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
      xtrack.row(16) = -20 * Eigen::VectorXd::Ones(NumberofKnotPt + 1); 


      /* ------------------------------------------------------------------------------------------------------ */

      /* ---------------------------------- State and Control Limits ---------------------------------- */
      ADMM::Saturation LIMITS;
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

      /* State Tracking. Force tracking */
      Eigen::MatrixXd F(3, N + 1);
      F.setZero();
      // F.row
      // xtrack.block(14, 0, 3, xtrack.cols()) = F;

      Eigen::MatrixXd R(3,3);
      R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
      double Tf = 2 * M_PI;


      std::vector<Eigen::MatrixXd> cartesianPoses = IK_traj.generateLissajousTrajectories(R, 0.8, 1, 3, 0.08, 0.08, N, Tf);

      /* initialize xinit, xgoal, xtrack - for the hozizon*/
      Eigen::VectorXd thetalist0(7);
      Eigen::VectorXd thetalistd0(7);
      Eigen::VectorXd q_bar(7);
      Eigen::VectorXd qd_bar(7);
      Eigen::VectorXd thetalist_ret(7);
      thetalist0 << 0.1, 0.2, 0.1, 0.2, 0.1, 0.1, 0.1;
      thetalistd0 << 0, 0, 0, 0, 0, 0, 0;
      q_bar << 0, 0, 0, 0, 0, 0, 0;
      qd_bar << 0, 0, 0, 0, 0, 0, 0;

      bool initial = true;
      IK_FIRST_ORDER IK = IK_FIRST_ORDER(IK_OPT.Slist,  IK_OPT.M, IK_OPT.joint_limits, IK_OPT.eomg, IK_OPT.ev, rho_init);

      IK.getIK(cartesianPoses.at(0), thetalist0, thetalistd0, q_bar, qd_bar, initial, rho_init, &thetalist_ret);
      xinit.head(7) = thetalist_ret;

      Eigen::VectorXd rho(5);
      rho << 20, 0.01, 0, 0, 1;

      commandVecTab_t u_0;
      u_0.resize(commandSize, N);
      u_0.setZero();

      std::cout << xtrack << std::endl;

      optimizerADMM.solve(xinit, u_0, xtrack, cartesianPoses, rho, LIMITS);


    }


protected:
    unsigned int N;
    double dt;
};



int main(int argc, char *argv[]) 
{
 
  unsigned int N = NumberofKnotPt;
  int ADMMiterMax = 5;
  double dt = TimeStep;

  ADMM::ADMMopt ADMM_OPTS(dt, 1e-7, 1e-7, 15, ADMMiterMax);

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


  /* -------------------- orocos kdl robot initialization-------------------------*/
  KUKAModelKDLInternalData robotParams;
  robotParams.numJoints = NDOF;
  robotParams.Kv = Eigen::MatrixXd(7,7);
  robotParams.Kp = Eigen::MatrixXd(7,7);


  /*------------------initialize control input-----------------------*/



  /* -------------------- Optimizer Params ------------------------ */
  optimizer::ILQRSolverADMM::OptSet solverOptions;
  solverOptions.n_hor    = N;
  solverOptions.tolFun   = ADMM_OPTS.tolFun;
  solverOptions.tolGrad  = ADMM_OPTS.tolGrad;
  solverOptions.max_iter = iterMax;


  /* ---------------------------------- Define the robot and contact model ---------------------------------- */
  KDL::Chain robot = KDL::KukaDHKdl();
  std::shared_ptr<RobotAbstract> kukaRobot = std::shared_ptr<RobotAbstract>(new KUKAModelKDL(robot, robotParams));

  FULL_ADMM admm = FULL_ADMM(N, TimeStep);
  admm.run(kukaRobot, solverOptions, ADMM_OPTS, IK_OPT);



  return 0;
}
