#ifndef ADMMMPCMAIN_H
#define ADMMMPCMAIN_H

#include <memory>

/* MPC trajectory generation */

#include <iostream>
#include <cmath>
#include <vector>
#include <cstdio>
#include <string>


#include "ADMMMultiBlock.hpp"
#include "plant.hpp"

#include "modern_robotics.h"
#include "differential_ik_trajectory.hpp"
#include "differential_ik_solver.hpp"
#include "robot_dynamics.hpp"
#include "RobotPublisherMPC.hpp"
#include "admm_public.hpp"
#include "ModelPredictiveControlADMM.hpp"



template<typename RobotModelOptimizer, typename RobotModel, int S, int C>
class ADMMTrajOptimizerMPC {

public:
    using Scalar            = double;
	using State 		    = Eigen::Matrix<Scalar, S, 1>;
    using Control           = Eigen::Matrix<Scalar, C, 1>;
    using DynamicsOptimizer = admm::Dynamics<RobotModelOptimizer, S, C>;
    using DynamicsPlant     = admm::Dynamics<RobotModel, S, C>;
	using PlantPublisher    = RobotPublisherMPC<Plant<DynamicsPlant, S, C>, S, C>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ADMMTrajOptimizerMPC() = default;
    ~ADMMTrajOptimizerMPC() = default;

    void run(const std::shared_ptr<RobotModelOptimizer>& kuka_model_optimizer,  std::shared_ptr<PlantPublisher>& plant_publisher, const State& init_state, const ContactModel::SoftContactModel<double>& contact_model,
        const ADMM_MPCconfig& ADMM_MPC_config, const TrajectoryDesired<S, NumberofKnotPt>& desired_trajectory, optimizer::IterativeLinearQuadraticRegulatorADMM::traj& result)
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
        std::shared_ptr<DynamicsOptimizer> KukaDynModel = std::shared_ptr<DynamicsOptimizer>(new RobotDynamics(dt, horizon_mpc, kuka_model_optimizer, contact_model));

        // Initialize Cost Function 
        std::shared_ptr<CostFunctionADMM> costFunction_admm = std::make_shared<CostFunctionADMM>(horizon_mpc, kuka_model_optimizer);

        // Optimizer Params 
        optimizer::IterativeLinearQuadraticRegulatorADMM::OptSet solverOptions;
        solverOptions.n_hor    = horizon_mpc; // not being used
        solverOptions.tolFun   = ADMM_OPTS.tolFun;
        solverOptions.tolGrad  = ADMM_OPTS.tolGrad;
        solverOptions.max_iter = iterMax;

        using OptimizerDDP = optimizer::IterativeLinearQuadraticRegulatorADMM;
        std::shared_ptr<OptimizerDDP> solver = std::make_shared<OptimizerDDP>(KukaDynModel, costFunction_admm, solverOptions, horizon_mpc, dt, ENABLE_FULLDDP, ENABLE_QPBOX);

        // admm optimizer
        ADMMMultiBlock<RobotModelOptimizer, RobotModel, S, C> optimizerADMM(kuka_model_optimizer, costFunction_admm, solver, ADMM_OPTS, IK_OPT, horizon_mpc);

        stateVec_t xinit = init_state;

        int iterations = 10;

        Eigen::VectorXd rho(5);
        rho << 150, 0.00, 0.00001, 0, 2;


        commandVecTab_t u_0;
        u_0.resize(commandSize, horizon_mpc);
        u_0.setZero();


        // MPC

        // Initialize receding horizon controller
        bool verbose = true;
        using Optimizer = ADMMMultiBlock<RobotModelOptimizer, RobotModel, S, C>;
        using Result    = optimizer::IterativeLinearQuadraticRegulatorADMM::traj;

        // Robot Publisher 
        using CostFunction   = std::shared_ptr<CostFunctionADMM>;

        ModelPredictiveControllerADMM<PlantPublisher, CostFunction, Optimizer, Result> mpc_admm(dt, horizon_mpc,
                                                                                                    iterations, 
                                                                                                    verbose, logger, 
                                                                                                    costFunction_admm,
                                                                                                    optimizerADMM, 
                                                                                                    desired_trajectory, 
                                                                                                    IK_OPT) ;


        /* lambda function - termination condition */
        using StateRef = Eigen::Ref<const stateVec_t>;
        auto termination =
        [&](int i, const StateRef &x)
        {
            auto N_ = 2700  - i;
            // auto N_ = (int)NFullTrajectory  - i;
            if (N_ <= 0) {
                return 1;
            } else {
                return 0;
            }
        };


        joint_state_traj.resize(S, NFullTrajectory + 1);


        // run MPC
        mpc_admm.run(xinit, u_0, plant_publisher, joint_state_traj, termination, rho, LIMITS);


        std::cout << "MPC_ADMM Trajectory Generation Finished! " << std::endl;

        delete(logger);

    }

    optimizer::IterativeLinearQuadraticRegulatorADMM::traj getOptimizerResult()
    {
        return resultTrajectory;
    }

protected:
	optimizer::IterativeLinearQuadraticRegulatorADMM::traj resultTrajectory;
    Eigen::MatrixXd joint_state_traj;

};

#endif

