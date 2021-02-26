
/* MPC trajectory generation */

#include "ADMMTrajOptimizerMPC.hpp"
#include "ModelPredictiveControlADMM.hpp"
#include "RobCodGenModel.h"

using Result = optimizer::IterativeLinearQuadraticRegulatorADMM::traj;

ADMMTrajOptimizerMPC::ADMMTrajOptimizerMPC() = default;
ADMMTrajOptimizerMPC::~ADMMTrajOptimizerMPC() = default;

void ADMMTrajOptimizerMPC::run(const std::shared_ptr<RobotAbstract>& kukaRobot, std::shared_ptr<ADMMTrajOptimizerMPC::PlantPublisher>& plantPublisher,  const stateVec_t& init_state,  const ContactModel::SoftContactModel<double>& contactModel,
  const ADMM_MPCconfig& ADMM_MPC_config, const TrajectoryDesired<stateSize, NumberofKnotPt>& desiredTrajectory,  Result& result) 
{

    uint NFullTrajectory = NumberofKnotPt;              
    Logger* logger = new DefaultLogger();


    // initialize control input
    unsigned int horizon_mpc = ADMM_MPC_config.horizon_;          // make these loadable from a cfg file
    auto ADMM_OPTS           = ADMM_MPC_config.ADMM_MPC_opt_.ADMM_opt_;
    auto dt                  = ADMM_MPC_config.dt_;
    unsigned int iterMax     = ADMM_MPC_config.ADMM_MPC_opt_.ADMM_opt_.iterMax;    
    auto IK_OPT              = ADMM_MPC_config.IK_opt_;
    auto LIMITS              = ADMM_MPC_config.ADMM_MPC_opt_.limits_;



    // Initialize Robot Model
    std::shared_ptr<Dynamics> KukaDynModel = std::shared_ptr<Dynamics>(new RobotDynamics(dt, horizon_mpc, kukaRobot, contactModel));

    // Initialize Cost Function 
    std::shared_ptr<CostFunctionADMM> costFunction_admm = std::make_shared<CostFunctionADMM>(horizon_mpc, kukaRobot);

    // initialize iLQR solver
    // Optimizer Params 
    optimizer::IterativeLinearQuadraticRegulatorADMM::OptSet solverOptions;
    solverOptions.n_hor    = horizon_mpc; // not being used
    solverOptions.tolFun   = ADMM_OPTS.tolFun;
    solverOptions.tolGrad  = ADMM_OPTS.tolGrad;
    solverOptions.max_iter = iterMax;

    using OptimizerDDP = optimizer::IterativeLinearQuadraticRegulatorADMM;
    std::shared_ptr<OptimizerDDP> solver = std::make_shared<OptimizerDDP>(KukaDynModel, costFunction_admm, solverOptions, horizon_mpc, dt, ENABLE_FULLDDP, ENABLE_QPBOX);

    // admm optimizer
    ADMMMultiBlock optimizerADMM(kukaRobot, costFunction_admm, solver, ADMM_OPTS, IK_OPT, horizon_mpc);

    stateVec_t xinit = init_state;

    int iterations = 10;

    Eigen::VectorXd rho(5);
    rho << 50, 0.00, 0.00001, 0, 2;


    commandVecTab_t u_0;
    u_0.resize(commandSize, horizon_mpc);
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
                                                                                                desiredTrajectory, 
                                                                                                IK_OPT) ;


    /* lambda function - termination condition */
    using StateRef = Eigen::Ref<const stateVec_t>;
    auto termination =
    [&](int i, const StateRef &x)
    {
        // auto N_ = 1800  - i;
        auto N_ = (int)NFullTrajectory  - i;
        if (N_ <= 0) {
            return 1;
        } else {
            return 0;
        }
    };


    joint_state_traj.resize(stateSize, NFullTrajectory + 1);


    /* ----------------------------------------- run MPC ------------------------- ------------------------- */
    mpc_admm.run(xinit, u_0, plantPublisher, joint_state_traj, termination, rho, LIMITS);


    std::cout << "MPC_ADMM Trajectory Generation Finished! " << std::endl;

    // delete(logger);

}

optimizer::IterativeLinearQuadraticRegulatorADMM::traj ADMMTrajOptimizerMPC::getOptimizerResult() 
{
  return resultTrajectory;
}





