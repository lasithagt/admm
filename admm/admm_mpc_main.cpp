
/* MPC trajectory generation */

#include "admm_mpc_main.hpp"


using namespace std;

void generateCartesianTrajectory(stateVec_t& xinit, stateVec_t& xgoal, stateVecTab_t& xtrack, std::vector<Eigen::MatrixXd> &cartesianPoses);


MPC_ADMM::MPC_ADMM() {}
MPC_ADMM::~MPC_ADMM() {}

void MPC_ADMM::run(std::shared_ptr<RobotAbstract>& kukaRobot, ContactModel::SoftContactModel& contactModel, const stateVec_t& xinit, const stateVecTab_t& xtrack, std::vector<Eigen::MatrixXd> &cartesianPoses, optimizer::ILQRSolverADMM::traj& result) 
{

    struct timeval tbegin,tend;
    double texec = 0.0;

    double dt      = TimeStep;
    unsigned int N = NumberofKnotPt;
    double tolFun  = 1e-5;                 // 1e-5; //relaxing default value: 1e-10; - reduction exit crieria
    double tolGrad = 1e-10;                // relaxing default value: 1e-10; - gradient exit criteria
    unsigned int iterMax = 5;              // 100;
    Logger* logger = new DefaultLogger();


    /*------------------initialize control input----------------------- */
    commandVecTab_t u_0;
    u_0.resize(commandSize, N);
    u_0.setZero();

    int horizon_mpc   = 10;          // make these loadable from a cfg file
    unsigned int temp_N = 10;



    int ADMMiterMax = 5;
    ADMM::ADMMopt ADMM_OPTS(dt, 1e-7, 1e-7, 15, ADMMiterMax);
    Eigen::MatrixXd joint_lims(2,7);
    double eomg = 0.00001;
    double ev   = 0.00001;

    /* -------------------- Optimizer Params ------------------------ */
    optimizer::ILQRSolverADMM::OptSet solverOptions;
    solverOptions.n_hor    = temp_N; // not being used
    solverOptions.tolFun   = ADMM_OPTS.tolFun;
    solverOptions.tolGrad  = ADMM_OPTS.tolGrad;
    solverOptions.max_iter = iterMax;


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


    /* ------------------------------------------------------------------------------------------------------ */

    /* --------------------------------------- State and Control Limits ------------------------------------- */
    ADMM::Saturation LIMITS;
    Eigen::VectorXd x_limits_lower(stateSize);
    Eigen::VectorXd x_limits_upper(stateSize);
    Eigen::VectorXd u_limits_lower(commandSize);
    Eigen::VectorXd u_limits_upper(commandSize);
    x_limits_lower << -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -10, -10, -10;    
    x_limits_upper << M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, M_PI, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 10, 10, 10;      
    u_limits_lower << -20, -20, -20, -20, -20, -20, -20;
    u_limits_upper << 20, 20, 20, 20, 20, 20, 20;

    LIMITS.stateLimits.row(0)   = x_limits_lower;
    LIMITS.stateLimits.row(1)   = x_limits_upper;
    LIMITS.controlLimits.row(0) = u_limits_lower; 
    LIMITS.controlLimits.row(1) = u_limits_upper; 

    /* ------------------------------------------------------------------------------------------------------ */

    // parameters for ADMM, penelty terms. initial
    Eigen::VectorXd rho_init(5);
    rho_init << 0, 0, 0, 0, 0;


    // penelty parameters
    Eigen::VectorXd rho(5);
    rho << 20, 0.01, 0, 0, 1;

    gettimeofday(&tbegin,NULL);


    // Initialize Robot Model
    KukaArm KukaArmModel(dt, temp_N, kukaRobot, contactModel);

    // Initialize Cost Function 
    CostFunctionADMM costFunction_admm(temp_N, kukaRobot);

    // initialize iLQR solver
    optimizer::ILQRSolverADMM solver(KukaArmModel, costFunction_admm, solverOptions, temp_N, dt, ENABLE_FULLDDP, ENABLE_QPBOX);

    // admm optimizer
    ADMM optimizerADMM(kukaRobot, costFunction_admm, solver, ADMM_OPTS, IK_OPT, temp_N);


    /* --------------------------- Plant -----------------------------------*/

    double state_var   = 0.0000001;
    double control_var = 0.0000001;

    KukaPlant<KukaArm, stateSize, commandSize> KukaModelPlant(KukaArmModel, dt, state_var, control_var);

    /* ---------------------------- MPC ----------------------------------- */

    // Initialize receding horizon controller
    bool verbose = true;
    using Plant = KukaPlant<KukaArm, stateSize, commandSize>;
    using Optimizer = ADMM;
    using Result = optimizer::ILQRSolverADMM::traj;


    int iterations = 10;
    int HMPC       = 10;



    ModelPredictiveControllerADMM<KukaArm, Plant, CostFunctionADMM, Optimizer, Result> mpc_admm(dt, horizon_mpc, HMPC,
     iterations, verbose, logger, KukaArmModel, costFunction_admm, optimizerADMM, xtrack, cartesianPoses) ;



    // termination condition
    using StateRef = Eigen::Ref<const stateVec_t>;
    auto termination =
    [&](int i, const StateRef &x)
    {
        auto N_ = N - (horizon_mpc+i);
        if (N_ <= 0) {
            return 1;
        } else {
            return 0;
        }
    };


    joint_state_traj.resize(stateSize, N + 1);


    // run MPC
    mpc_admm.run(xinit, u_0.block(0, 0, commandSize, horizon_mpc), KukaModelPlant, joint_state_traj, termination, rho, LIMITS);


    gettimeofday(&tend,NULL);


    texec=(static_cast<double>(1000*(tend.tv_sec-tbegin.tv_sec)+((tend.tv_usec-tbegin.tv_usec)/1000)))/1000.;

    cout << endl;
    cout << "Number of iterations: " << lastTraj.iter + 1 << endl;
    cout << "Final cost: " << lastTraj.finalCost << endl;
    cout << "Final gradient: " << lastTraj.finalGrad << endl;
    cout << "Final lambda: " << lastTraj.finalLambda << endl;
    cout << "Execution time by time step (second): " << texec/N << endl;
    cout << "Execution time per iteration (second): " << texec/lastTraj.iter << endl;
    cout << "Total execution time of the solver (second): " << texec << endl;
    cout << "\tTime of derivative (second): " << lastTraj.time_derivative.sum() << " (" << 100.0*lastTraj.time_derivative.sum()/texec << "%)" << endl;
    cout << "\tTime of backward pass (second): " << lastTraj.time_backward.sum() << " (" << 100.0*lastTraj.time_backward.sum()/texec << "%)" << endl;




    cout << "------------------------------------ MPC_ADMM Trajectory Generation Finished! ------------------------------------" << endl;

    delete(logger);

}



// Generate cartesian trajectory
void generateCartesianTrajectory(stateVec_t& xinit, stateVec_t& xgoal, stateVecTab_t& xtrack, std::vector<Eigen::MatrixXd> &cartesianPoses) {
    Eigen::MatrixXd joint_lims(2,7);
    double eomg = 0.00001;
    double ev   = 0.00001;
    unsigned int N = NumberofKnotPt;

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

    // this is for admm parameters. For normal ddp, set it to zero.
    Eigen::VectorXd rho_init(5);
    rho_init << 0, 0, 0, 0, 0;

    IKTrajectory<IK_FIRST_ORDER> IK_traj = IKTrajectory<IK_FIRST_ORDER>(Slist, M, joint_lims, eomg, ev, rho_init, N);

    Eigen::MatrixXd R(3,3);
    R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    double Tf = 2 * M_PI;

    cartesianPoses = IK_traj.generateLissajousTrajectories(R, 0.8, 1, 3, 0.08, 0.08, N, Tf);


    /* initialize xinit, xgoal, xtrack - for the hozizon*/
    Eigen::MatrixXd joint_trajectory(7, N + 1);
    Eigen::VectorXd thetalist0(7), thetalistd0(7), thetalist_ret(7);
    thetalist0  << 0.1, 0.2, 0.1, 0.2, 0.1, 0.1, 0.1;
    thetalistd0 << 0, 0, 0, 0, 0, 0, 0;


    bool initial = true;
    IK_FIRST_ORDER IK = IK_FIRST_ORDER(IK_OPT.Slist,  IK_OPT.M, IK_OPT.joint_limits, IK_OPT.eomg, IK_OPT.ev, rho_init);

    IK.getIK(cartesianPoses.at(0), thetalist0, thetalistd0, Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7), initial, rho_init, &thetalist_ret);
    xinit.head(7) = thetalist_ret;

}


int main(int argc, char *argv[]) 
{
 
    MPC_ADMM optimizerADMM;
    stateVec_t xinit, xgoal;
    stateVecTab_t xtrack;
    xtrack.resize(stateSize, NumberofKnotPt + 1);

    std::vector<Eigen::MatrixXd> cartesianPoses;
    generateCartesianTrajectory(xinit, xgoal, xtrack, cartesianPoses);

    xtrack.row(16) = 5 * Eigen::VectorXd::Ones(NumberofKnotPt + 1); 


    /* -------------------- orocos kdl robot initialization-------------------------*/
    KUKAModelKDLInternalData robotParams;
    robotParams.numJoints = NDOF;
    robotParams.Kv = Eigen::MatrixXd(7,7);
    robotParams.Kp = Eigen::MatrixXd(7,7);

    // robot instance initialization
    KDL::Chain robot = KDL::KukaDHKdl();
    std::shared_ptr<RobotAbstract> kukaRobot = std::shared_ptr<RobotAbstract>(new KUKAModelKDL(robot, robotParams));
    
    // contact model dynamics
    ContactModel::ContactParams cp_;
    cp_.E = 1000;
    cp_.mu = 0.5;
    cp_.nu = 0.55;
    cp_.R  = 0.005;
    cp_.R_path = 1000;
    cp_.Kd = 10;
    ContactModel::SoftContactModel contactModel(cp_);
    kukaRobot->initRobot();

    optimizer::ILQRSolverADMM::traj result;
    optimizerADMM.run(kukaRobot, contactModel, xinit, xtrack, cartesianPoses, result);


  return 0;

}