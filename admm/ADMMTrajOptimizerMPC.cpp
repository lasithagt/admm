
/* MPC trajectory generation */

#include "ADMMTrajOptimizerMPC.hpp"
#include "admmPublic.hpp"
#include "ModelPredictiveControlADMM.hpp"
#include "RobCodGenModel.h"



ADMMTrajOptimizerMPC::ADMMTrajOptimizerMPC() = default;
ADMMTrajOptimizerMPC::~ADMMTrajOptimizerMPC() = default;

void ADMMTrajOptimizerMPC::run(const std::shared_ptr<RobotAbstract>& kukaRobot, std::shared_ptr<ADMMTrajOptimizerMPC::PlantPublisher>& plantPublisher,  const stateVec_t& init_state,  const ContactModel::SoftContactModel<double>& contactModel,  const ADMMopt& ADMM_OPTS, const IKTrajectory<IK_FIRST_ORDER>::IKopt& IK_OPT, \
    const Saturation& LIMITS, const std::vector<Eigen::MatrixXd>& cartesianPoses,  optimizer::IterativeLinearQuadraticRegulatorADMM::traj& result) 
{

    struct timeval tbegin,tend;
    double texec = 0.0;

    double dt      = TimeStep;
    unsigned int N = NumberofKnotPt;
    double tolFun  = 1e-5;                 
    double tolGrad = 1e-10;                
    unsigned int iterMax = 6;              
    Logger* logger = new DefaultLogger();


    /*------------------initialize control input----------------------- */
    unsigned int horizon_mpc = 100;          // make these loadable from a cfg file


    // Initialize Robot Model
    using Dynamics = admm::Dynamics<RobotAbstract, stateSize, commandSize>;
    std::shared_ptr<Dynamics> KukaDynModel = std::shared_ptr<Dynamics>(new RobotDynamics(dt, horizon_mpc, kukaRobot, contactModel));

    // Initialize Cost Function 
    std::shared_ptr<CostFunctionADMM> costFunction_admm = std::make_shared<CostFunctionADMM>(horizon_mpc, kukaRobot);

    // initialize iLQR solver
    /* -------------------- Optimizer Params ------------------------ */
    optimizer::IterativeLinearQuadraticRegulatorADMM::OptSet solverOptions;
    solverOptions.n_hor    = horizon_mpc; // not being used
    solverOptions.tolFun   = ADMM_OPTS.tolFun;
    solverOptions.tolGrad  = ADMM_OPTS.tolGrad;
    solverOptions.max_iter = iterMax;

    using OptimizerDDP = optimizer::IterativeLinearQuadraticRegulatorADMM;
    std::shared_ptr<OptimizerDDP> solver = std::make_shared<OptimizerDDP>(KukaDynModel, costFunction_admm, solverOptions, horizon_mpc, dt, ENABLE_FULLDDP, ENABLE_QPBOX);

    // admm optimizer
    ADMMMultiBlock optimizerADMM(kukaRobot, costFunction_admm, solver, ADMM_OPTS, IK_OPT, horizon_mpc);



    stateVec_t xinit;
    stateVecTab_t xtrack;
    xtrack.resize(stateSize, N + 1);
    xtrack.row(16) = 5 * Eigen::VectorXd::Ones(N + 1); 


    /* initialize xinit, xgoal, xtrack - for the horizon*/
    Eigen::VectorXd thetalist0(7);
    Eigen::VectorXd thetalistd0(7);
    Eigen::VectorXd q_bar(7);
    Eigen::VectorXd qd_bar(7);
    Eigen::VectorXd thetalist_ret(7);


    for (int i = 0;i < 7; i++) { thetalist0(i) = init_state(i);}

    thetalistd0 << 0, 0, 0, 0, 0, 0, 0;
    q_bar << 0, 0, 0, 0, 0, 0, 0;
    qd_bar << 0, 0, 0, 0, 0, 0, 0;


    Eigen::VectorXd rho_init(5);
    rho_init << 0, 0, 0, 0, 0;
    bool initial = true;
    IK_FIRST_ORDER IK = IK_FIRST_ORDER(IK_OPT.Slist,  IK_OPT.M, IK_OPT.joint_limits, IK_OPT.eomg, IK_OPT.ev, rho_init);

    IK.getIK(cartesianPoses.at(0), thetalist0, thetalistd0, q_bar, qd_bar, initial, rho_init, thetalist_ret);
    xinit.head(7) = thetalist_ret;

    // xinit = init_state;

    int iterations = 10;

    /* ------------------------------------------------ Penelty parameters ----------------------------------------------------- */
    Eigen::VectorXd rho(5);
    rho << 50, 0.1, 0.00001, 0, 2;


    commandVecTab_t u_0;
    u_0.resize(commandSize, N);
    u_0.setZero();


    /* ------------------------------------------------------ MPC ---------------------------------------------------------------- */

    // Initialize receding horizon controller
    bool verbose = true;
    using Optimizer = ADMMMultiBlock;
    using Result    = optimizer::IterativeLinearQuadraticRegulatorADMM::traj;

    /* ------------------------------------------------- Robot Publisher ----------------------------------------------------------*/
    using RobotPublisher = ADMMTrajOptimizerMPC::PlantPublisher;
    using CostFunction   = std::shared_ptr<CostFunctionADMM>;

    ModelPredictiveControllerADMM<RobotPublisher, CostFunction, Optimizer, Result> mpc_admm(dt, horizon_mpc,
                                                                                                iterations, 
                                                                                                verbose, logger, 
                                                                                                costFunction_admm,
                                                                                                optimizerADMM, 
                                                                                                xtrack, cartesianPoses, 
                                                                                                IK_OPT) ;


    /* lambda function - termination condition */
    using StateRef = Eigen::Ref<const stateVec_t>;
    auto termination =
    [&](int i, const StateRef &x)
    {
        auto N_ = (int)N  - i;
        // auto N_ = (int)N - (1 + (int)horizon_mpc + i);
        if (N_ <= 0) {
            return 1;
        } else {
            return 0;
        }
    };


    joint_state_traj.resize(stateSize, N + 1);


    /* ----------------------------------------- run MPC ------------------------- ------------------------- */
    mpc_admm.run(xinit, u_0.block(0, 0, commandSize, horizon_mpc), plantPublisher, joint_state_traj, termination, rho, LIMITS);

    std::cout << "MPC_ADMM Trajectory Generation Finished! " << std::endl;

    delete(logger);

}

optimizer::IterativeLinearQuadraticRegulatorADMM::traj ADMMTrajOptimizerMPC::getOptimizerResult() 
{
  return resultTrajectory;
}





