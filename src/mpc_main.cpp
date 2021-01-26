
#include <memory>

/* MPC trajectory generation */

#include "mpc.hpp"
#include "plant.h"

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <fstream>
#include <string>
#include <list>

#include <Eigen/Dense>

#include "config.h"
#include "ilqrsolver.h"
#include "kuka_arm.h"
#include "SoftContactModel.h"
#include "KukaModel.h"
#include "models.h"

#include "modern_robotics.h"
#include "ik_trajectory.hpp"
#include "ik_solver.hpp"
#include "kuka_robot.hpp"


using namespace std;
using namespace Eigen;




class MPC {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MPC() {}
    ~MPC() {}

    void run(stateVec_t xinit, stateVec_t xgoal, const stateVecTab_t &xtrack) 
    {
        struct timeval tbegin,tend;
        double texec = 0.0;

        double dt = TimeStep;
        unsigned int N = NumberofKnotPt;
        double tolFun = 1e-5; // 1e-5;//relaxing default value: 1e-10; - reduction exit crieria
        double tolGrad = 1e-10; // relaxing default value: 1e-10; - gradient exit criteria
        unsigned int iterMax = 5; // 100;
        Logger* logger = new DefaultLogger();

        /* -------------------- orocos kdl robot initialization-------------------------*/
        KUKAModelKDLInternalData robotParams;
        robotParams.numJoints = NDOF;
        robotParams.Kv = Eigen::MatrixXd(7,7);
        robotParams.Kp = Eigen::MatrixXd(7,7);


        KDL::Chain robot = KDL::KukaDHKdl();
        std::shared_ptr<RobotAbstract> kukaRobot = std::shared_ptr<RobotAbstract>(new KUKAModelKDL(robot, robotParams));

        //======================================
        // contact model dynamics
        ContactModel::ContactParams cp_;
        cp_.E = 1000;
        cp_.mu = 0.5;
        cp_.nu = 0.4;
        cp_.R  = 0.005;
        cp_.R_path = 1000;
        cp_.Kd = 10;
        ContactModel::SoftContactModel contactModel(cp_);
        kukaRobot->initRobot();



        /*------------------initialize control input----------------------- */
        commandVecTab_t u_0;
        u_0.resize(commandSize, N);
        u_0.setZero();

        int horizon_mpc   = 20;          // make these loadable from a cfg file
        unsigned int temp_N = 20;

        // Initialize Robot Model
        KukaArm KukaArmModel(dt, temp_N, kukaRobot, contactModel);

        // Initialize Cost Function 
        CostFunction costKukaArm(horizon_mpc);

        /* -------------------- Optimizer Params ------------------------ */
        optimizer::ILQRSolver::OptSet solverOptions;
        solverOptions.n_hor    = horizon_mpc; // not being used
        solverOptions.tolFun   = tolFun;
        solverOptions.tolGrad  = tolGrad;
        solverOptions.max_iter = iterMax;

        // initialize iLQR solver
        optimizer::ILQRSolver solver(KukaArmModel, costKukaArm, solverOptions, horizon_mpc, dt, ENABLE_FULLDDP, ENABLE_QPBOX);


        gettimeofday(&tbegin,NULL);


        /* --------------------------- Plant -----------------------------------*/

        double state_var   = 0.00001;
        double control_var = 0.00001;

        KukaPlant<KukaArm, stateSize, commandSize> KukaModelPlant(KukaArmModel, dt, state_var, control_var);

        // stateVec_t state = KukaModelPlant.f(s, c);

        /* ---------------------------- MPC ----------------------------------- */

        // Initialize receding horizon controller
        bool verbose = true;
        using Plant = KukaPlant<KukaArm, stateSize, commandSize>;
        using Optimizer = optimizer::ILQRSolver;
        using Result = optimizer::ILQRSolver::traj;


        int iterations = 10;
        int HMPC       = 10;
        ModelPredictiveController<KukaArm, Plant, CostFunction, Optimizer, Result> mpc(dt, horizon_mpc, HMPC,
         iterations, verbose, logger, KukaArmModel, costKukaArm, solver, xtrack) ;

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
        mpc.run(xinit, u_0.block(0, 0, commandSize, horizon_mpc), KukaModelPlant, joint_state_traj, termination);


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




        cout << "------------------------------------ MPC Trajectory Generation Finished! ------------------------------------" << endl;

        delete(logger);

    }
private:
    Eigen::MatrixXd joint_state_traj;
    commandVecTab_t torque_traj;
    stateVecTab_t joint_state_traj_interp;
    commandVecTab_t torque_traj_interp;

protected:
    optimizer::ILQRSolver::traj lastTraj;
};



// Generate cartesian trajectory
void generateCartesianTrajectory(stateVec_t& xinit, stateVec_t& xgoal, stateVecTab_t& xtrack) {
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

    std::vector<Eigen::MatrixXd> cartesianPoses = IK_traj.generateLissajousTrajectories(R, 0.8, 1, 3, 0.08, 0.08, N, Tf);


    /* initialize xinit, xgoal, xtrack - for the hozizon*/
    Eigen::MatrixXd joint_trajectory(7, N + 1);
    Eigen::VectorXd thetalist0(7), thetalistd0(7), thetalist_ret(7);
    thetalist0  << 0.1, 0.2, 0.1, 0.2, 0.1, 0.1, 0.1;
    thetalistd0 << 0, 0, 0, 0, 0, 0, 0;


    bool initial = true;
    IK_FIRST_ORDER IK = IK_FIRST_ORDER(IK_OPT.Slist,  IK_OPT.M, IK_OPT.joint_limits, IK_OPT.eomg, IK_OPT.ev, rho_init);

    IK.getIK(cartesianPoses.at(0), thetalist0, thetalistd0, Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7), initial, rho_init, thetalist_ret);
    xinit.head(7) = thetalist_ret;

    // IK trajectory initialization
    IKTrajectory<IK_FIRST_ORDER> IK_solve = IKTrajectory<IK_FIRST_ORDER>(IK_OPT.Slist, IK_OPT.M, 
    IK_OPT.joint_limits, IK_OPT.eomg, IK_OPT.ev, rho_init, N);

    IK_solve.getTrajectory(cartesianPoses, xinit.col(0).head(7), xinit.col(0).segment(7, 7), 
    Eigen::MatrixXd::Zero(7, N + 1), Eigen::MatrixXd::Zero(7, N + 1), rho_init, &joint_trajectory);


    xtrack.block(0, 0, 7, N + 1) = joint_trajectory;
    xgoal.head(7) = joint_trajectory.col(N).head(7);

}


int main(int argc, char *argv[]) 
{
 
    MPC optimizer;
    stateVec_t xinit, xgoal;
    stateVecTab_t xtrack;
    xtrack.resize(stateSize, NumberofKnotPt + 1);


    generateCartesianTrajectory(xinit, xgoal, xtrack);


    xtrack.row(16) = 5 * Eigen::VectorXd::Ones(NumberofKnotPt + 1); 

    KUKAModelKDLInternalData robotParams;
    robotParams.numJoints = 7;
    robotParams.Kv = Eigen::MatrixXd(7,7);
    robotParams.Kp = Eigen::MatrixXd(7,7);
    

    optimizer.run(xinit, xgoal, xtrack);


    /* TODO : publish to the robot */
  // e.g. ROS, drake, bullet


  return 0;
}



