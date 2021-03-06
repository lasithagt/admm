/// @file
///
/// kuka_plan_runner is designed to wait for LCM messages contraining
/// a robot_plan_t message, and then execute the plan on an iiwa arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.

#include <iostream>
#include <memory>

/* DDP trajectory generation */

#include "ddp.h"
#include "cnpy.h"


using namespace std;
using namespace Eigen;

DDP::DDP()
{

}

/* -------------------- Soft_contact_state = 17(14+3) ------------------------*/
void DDP::run(stateVec_t xinit, stateVec_t xgoal, stateVecTab_t xtrack) 
{
    struct timeval tbegin,tend;
    double texec = 0.0;

    double dt            = TimeStep;
    unsigned int N       = NumberofKnotPt;
    double tolFun        = 1e-7;       // 1e-5;//relaxing default value: 1e-10; - reduction exit crieria
    double tolGrad       = 1e-10;     // relaxing default value: 1e-10; - gradient exit criteria
    unsigned int iterMax = 15;  // 100;


    /* -------------------- orocos kdl robot initialization-------------------------*/
    KUKAModelKDLInternalData robotParams;
    robotParams.numJoints = 7;
    robotParams.Kv = Eigen::MatrixXd(7,7);
    robotParams.Kp = Eigen::MatrixXd(7,7);


    KDL::KukaDHKdl robot = KDL::KukaDHKdl();
    KDL::Chain chain = robot();
    std::shared_ptr<RobotAbstract> kukaRobot = std::shared_ptr<RobotAbstract>(new KUKAModelKDL(chain, robotParams));

    //======================================

    ContactModel::ContactParams<double> cp_;
    cp_.E = 1000;
    cp_.mu = 0.5;
    cp_.nu = 0.4;
    cp_.R  = 0.005;
    cp_.R_path = 1000;
    cp_.Kd = 10;
    ContactModel::SoftContactModel<double> contactModel(cp_);
    kukaRobot->initRobot();

    // get the initial goal pose
    // kukaRobot->getForwardKinematics(q_pos_init.data(), q_vel_init.data(), qdd.data(), poseM, poseP, vel, accel, false);
    // Eigen::Matrix3d m;
    // m = Eigen::AngleAxisd(poseM);
    // Eigen::Vector3d ea = m.eulerAngles(2, 1, 0); 

    // // get the final goal pose
    // kukaRobot->getForwardKinematics(q_pos_goal.data(), q_vel_goal.data(), qdd.data(), poseM, poseP, vel, accel, false);
    // m = Eigen::AngleAxisd(poseM);
    // ea = m.eulerAngles(2, 1, 0); 


    /*----------------------warm-start-control-----------------------------*/
    // Get the gravity compensation for warm-start

    // qd.setZero();
    // Eigen::VectorXd gravityTorque(7);
    // kukaRobot->getGravityVector(q_pos_init.data(), gravityTorque);


    /*------------------initialize control input----------------------- */
    commandVecTab_t u_0;
    u_0.resize(commandSize, N);
    u_0.setZero();


    // robot model
    RobotDynamics KukaArmModel(dt, N, kukaRobot, contactModel);

    // cost function 
    CostFunction costKukaArm(N);

    /* -------------------- Optimizer Params ------------------------ */
    optimizer::ILQRSolver::OptSet solverOptions;
    solverOptions.n_hor    = N;
    solverOptions.tolFun   = tolFun;
    solverOptions.tolGrad  = tolGrad;
    solverOptions.max_iter = iterMax;

    // iLQR solver
    optimizer::ILQRSolver solver(KukaArmModel, costKukaArm, solverOptions, N, dt, ENABLE_FULLDDP, ENABLE_QPBOX);
    // solver.firstInitSolver(N, dt);   


    gettimeofday(&tbegin, NULL);
    

    // Run iLQR 
    solver.solve(xinit, u_0, xtrack);
    

    /* ------------------------------------------------------------------- */

    gettimeofday(&tend,NULL);
    lastTraj = solver.getLastSolvedTrajectory();

    /* ------------------------------------------------------------------- */
    const int InterpolationScale = 10;
    /* post processing and save data */
    joint_state_traj.resize(stateSize, N + 1);
    joint_state_traj_interp.resize(stateSize, N * InterpolationScale + 1);

    for(unsigned int i=0; i <= N; i++)
    {
      joint_state_traj.col(i) = lastTraj.xList.col(i);
    }

    // save data
    const int Ny = joint_state_traj.rows();
    const int Nx = joint_state_traj.cols();

    cnpy::npy_save("../data/state_trajectory_ddp.npy", joint_state_traj.data(),{1, static_cast<unsigned long>(Nx), static_cast<unsigned long>(Ny)}, "w");
    cnpy::npy_save("../data/state_trajectory_ddp_desired.npy", xtrack.data(),{1, static_cast<unsigned long>(Nx), static_cast<unsigned long>(Ny)}, "w");


    torque_traj = lastTraj.uList;

    //linear interpolation to 1ms
    for (unsigned int i = 0;i < stateSize; i++)
    {
      for (unsigned int j = 0;j < N*InterpolationScale; j++)
      {
        unsigned int index = j / 10;
        joint_state_traj_interp.col(j)(i) =  joint_state_traj.col(index)(i) + (static_cast<double>(j) - static_cast<double>(index*10.0))*(joint_state_traj.col(index + 1)(i) - joint_state_traj.col(index)(i))/10.0;
      }
      joint_state_traj_interp.col(N * InterpolationScale)(i) = joint_state_traj.col(N)(i);
    }

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



    cout << "lastTraj.xList[" << N << "]:" << lastTraj.xList.col(N).transpose() << endl;
    cout << "lastTraj.uList[" << N << "]:" << lastTraj.uList.col(N-1).transpose() << endl;

    cout << "lastTraj.xList[0]:" << lastTraj.xList.col(0).transpose() << endl;
    cout << "lastTraj.uList[0]:" << lastTraj.uList.col(0).transpose() << endl;




    cout << "-------- DDP Trajectory Generation Finished! --------" << endl;

}






