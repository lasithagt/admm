#ifndef MPCADMM_H
#define MPCADMM_H

#include <memory>

#include "config.h"
#include "RobotDynamics.hpp"
#include "SoftContactModel.h"
#include "KukaModel.h"
#include "models.h"


#include <iostream>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "modern_robotics.h"
#include "DiffIKTrajectory.hpp"
#include "DiffIKSolver.hpp"
#include "cnpy.h"
#include "KukaKinematicsScrews.hpp"
#include "admmPublic.hpp"
#include "RobotPublisherMPC.hpp"


std::mutex mu_main;
std::condition_variable cv_main;
bool mpcComputeFinished = false;
bool currentStateReceived = false;


template <typename RobotPublisher>
void publishCommands(RobotPublisher& publisher)
{
	// run until optimizer is publishing
	while (!publisher.terminate) {
		{
			if (publisher.terminate) {mpcComputeFinished = true;}
		
			std::unique_lock<std::mutex> lk(mu_main);
			cv_main.wait(lk, []{return mpcComputeFinished;});

 

			publisher.m_robotPlant.applyControl(publisher.controlBuffer.col(0));
			std::cout << "Publishing Control Command..." << 0 <<  std::endl;


			// get current state
			publisher.currentState = publisher.m_robotPlant.getCurrentState();

			// store the states
			publisher.stateBuffer->col(0) = publisher.currentState;  

			currentStateReceived = true;
			mpcComputeFinished = false;
			
			
			cv_main.notify_one();
			
			lk.unlock();
			{
				for (int i = 1;i < 2;i++)
				{
					// publisher.m_robotPlant.applyControl(publisher.controlBuffer.col(i));
					std::this_thread::sleep_for(std::chrono::milliseconds(100));

					// get current state
					// publisher.currentState = publisher.m_robotPlant.getCurrentState();
					std::cout << "Publishing Control Command..." << i <<  std::endl;

					// store the states
					// lk.lock()
					// publisher.stateBuffer->col(i) = publisher.currentState;   
					// lk.unlock();
				}
			}
			

		}

		std::cout << "Finished Publishing Thread" << std::endl;

	}

}

template <class RobotPublisherT, class costFunctionT, class OptimizerT, class OptimizerResultT>
class ModelPredictiveControllerADMM
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Scalar                = double;            ///< Type of scalar used by the optimizer
    using Optimizer             = OptimizerT;
    using CostFunction   		= costFunctionT;
    using State                 = stateVec_t;            
    using Control               = commandVec_t;           
    using StateTrajectory       = stateVecTab_t;   
    using ControlTrajectory     = commandVecTab_t; 
    using RobotPublisher 	 	= RobotPublisherT;
    using Result                = OptimizerResultT;            ///< Type of result returned by the optimizer
    using TerminationCondition =
        std::function<bool(int, State)>;     ///< Type of termination condition function

    static const int MPC_BAD_CONTROL_TRAJECTORY = -1;
    Optimizer opt_;
    CostFunction& cost_function_;
    bool verbose_;
    Scalar dt_;
    int H_;
    Logger* logger_;
    ControlTrajectory control_trajectory;
    StateTrajectory x_track_;
    std::vector<Eigen::MatrixXd> cartesianTrack_;
    Eigen::MatrixXd cartesian_desired_logger;
    Eigen::MatrixXd cartesian_mpc_logger;
    int HMPC_;
    IKTrajectory<IK_FIRST_ORDER>::IKopt IK_OPT;
    Eigen::MatrixXd actual_cartesian_pose;


public:
    /**
     * @brief               Instantiate the receding horizon wrapper for the trajectory optimizer.
     * @param dt            Time step
     * @param time_steps    Number of time steps over which to optimize
     * @param iterations    The number of iterations to perform per time step
     * @param logger        util::Logger to use for informational, warning, and error messages
     * @param verbose       True if informational and warning messages should be passed to the logger; error messages are always passed
     * @param args          Arbitrary arguments to pass to the trajectory optimizer at initialization time
     */
    ModelPredictiveControllerADMM(Scalar dt, int time_steps, int HMPC, int iterations, bool verbose, Logger *logger, \
	         CostFunction                   &cost_function,
	         Optimizer 						&opt,
	         const StateTrajectory  		&x_track, 
	         const std::vector<Eigen::MatrixXd> &cartesianTrack,
    		 const IKTrajectory<IK_FIRST_ORDER>::IKopt& IK_OPT_)
    : dt_(dt), H_(time_steps), HMPC_(HMPC), verbose_(verbose), cost_function_(cost_function), opt_(opt), x_track_(x_track), cartesianTrack_(cartesianTrack), IK_OPT(IK_OPT_)
    {
    	logger_ = logger;
    	control_trajectory.resize(commandSize, H_);
    	cartesian_desired_logger.resize(3, cartesianTrack_.size());
    	cartesian_mpc_logger.resize(3, cartesianTrack_.size());

    	cartesian_mpc_logger.setZero();

    	for (int i=0;i<cartesianTrack_.size();i++) {
    	}

    	actual_cartesian_pose.resize(4,4);
    	
    }

    /**
     * @brief                               Run the trajectory optimizer in MPC mode.
     * @param initial_state                 Initial state to pass to the optimizer
     * @param initial_control_trajectory    Initial control trajectory to pass to the optimizer
     * @param terminate                     Termination condition to check before each time step
     * @param dynamics                      Dynamics model to pass to the optimizer
     * @param plant                         Plant model
     * @param cost_function                 Running cost function L(x, u) to pass to the optimizer
     * @param terminal_cost_function        Terminal cost function V(xN) to pass to the optimizer
     * @param args                          Arbitrary arguments to pass to the trajectory optimizer at run time
     */
    template <typename TerminationCondition>
	void run(const Eigen::Ref<const State>  &initial_state,
	         ControlTrajectory              initial_control_trajectory,
	         RobotPublisher                 &robotPublisher,
	         Eigen::MatrixXd	            &joint_state_traj, // save data
	         TerminationCondition           &terminate,
	         const Eigen::VectorXd 			&rho,
	         const Saturation			&L)
	         // TerminalCostFunction           &terminal_cost_function)
	{

	    if (initial_control_trajectory.cols() != H_)
	    {
	        logger_->error("The size of the control trajectory does not match the number of time steps passed to the optimizer!");
	        std::exit(MPC_BAD_CONTROL_TRAJECTORY);
	    }

	    State x = initial_state, xold = initial_state;
	    std::cout << xold << std::endl;
	    Control u;
	    // Scalar true_cost = cost_function.c(xold, initial_control_trajectory[0]);

	    Eigen::MatrixXd x_track_mpc;
	    std::vector<Eigen::MatrixXd> cartesianTrack_mpc;
	    cartesianTrack_mpc.resize(H_ + 1);

	    for (int k = 0;k < H_ + 1;k++) {
	    	cartesianTrack_mpc[k] = cartesianTrack_[k];
	    }

	    x_track_mpc.resize(stateSize, H_ + 1);
	    x_track_mpc = x_track_.block(0, 0, stateSize, H_ + 1);


	    scalar_t true_cost = cost_function_.cost_func_expre(0, xold, initial_control_trajectory.col(0), x_track_.col(0));
	    
	    Result result;
	    control_trajectory = initial_control_trajectory;
	    u = initial_control_trajectory.col(0);
	    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
	    std::chrono::duration<float, std::milli> elapsed;

	    Eigen::MatrixXd stateBuffer;
	    stateBuffer.resize(stateSize, HMPC_);

    	// to store the state evolution
    	robotPublisher.setStateBuffer(&stateBuffer);

    	robotPublisher.setInitialState(initial_state);

    	// call the thread
		std::thread robotPublishThread(publishCommands<RobotPublisher>, std::ref(robotPublisher));


 	    int64_t i = 0;
	    while(!terminate(i, x))
	    {

	        std::cout << "MPC loop started..." << std::endl;

	        if(verbose_)
	        {
	            if(i > 0)
	            {
	                end = std::chrono::high_resolution_clock::now();
	                elapsed = end - start;
	                logger_->info("Completed MPC loop for time step %d in %d ms\n", i - 1, static_cast<int>(elapsed.count()));
	            }
	            logger_->info("Entered MPC loop for time step %d\n", i);
	            start = std::chrono::high_resolution_clock::now();
	        }


	        // Run the optimizer to obtain the next control
	        {
		        opt_.solve(xold, control_trajectory, x_track_mpc, cartesianTrack_mpc, rho, L);
	    	}
	        
	        result = opt_.getLastSolvedTrajectory();
	        u = result.uList.col(0);
	        control_trajectory = result.uList;

	        if(verbose_)
	        {
	            logger_->info("Obtained control from optimizer: ");
	            for(int m = 0; m < u.rows(); ++m) { logger_->info("%f ", u(m)); }
	            logger_->info("\n");
	        }

	        // Apply the control to the plant and obtain the new state
	        x = xold; 


        	/* ----------------------------- apply to the plant. call a child thread ----------------------------- */
        	// set the control trajectory
        	robotPublisher.setControlBuffer(control_trajectory);

		    {
		        std::lock_guard<std::mutex> lk(mu_main);
		        mpcComputeFinished = true;
		        std::cout << "main() signals data ready for processing\n";
		    }

		    cv_main.notify_one();


	        for (int k = 0; k < HMPC_; k++) {
	        	/* apply to the plant */
			    {
			        std::unique_lock<std::mutex> lk(mu_main);
			        cv_main.wait(lk, []{return currentStateReceived;});
			    }

			    end = std::chrono::high_resolution_clock::now();
			    elapsed = end - start;

	        	x = stateBuffer.col(k);
	        	currentStateReceived = false;

	        	// save data
	        	#ifdef DEBUG
	        	cartesian_desired_logger.col(i + k) = cartesianTrack_.at(i + k).col(3).head(3);
			    // joint_state_traj.col(i + k) = x;
				actual_cartesian_pose = mr::FKinSpace(IK_OPT.M, IK_OPT.Slist, x.head(7));
				// std::cout << actual_cartesian_pose.col(3).head(3) << std::endl;
				cartesian_mpc_logger.col(i+k) = actual_cartesian_pose.col(3).head(3);
				// std::cout << actual_cartesian_pose.col(i) << std::endl;
			    #endif

	        }

	        if(verbose_)
	        {
	            logger_->info("Received new state from plant: ");
	            for(int n = 0; n < x.rows(); ++n) { logger_->info("%f ", x(n)); }
	            logger_->info("\n");
	        }

	        // Calculate the true cost for this time step
	        true_cost = cost_function_.cost_func_expre(0, xold, u, x_track_.col(i));

	        if(verbose_) logger_->info("True cost for time step %d: %f\n", i, true_cost);

	        // Slide down the control trajectory
	        // control_trajectory.leftCols(H_ - 1) = result.uList.rightCols(H_ - 1);


	        if(verbose_) logger_->info("Slide down the control trajectory\n");
	        // control_trajectory = result.uList;
	        control_trajectory = initial_control_trajectory;
	        
	        xold = x;


	        // slide down the control for state and cartesian state
	        x_track_mpc = x_track_.block(0, i, stateSize, H_ + 1);

	        for (int k = 0;k < H_+1;k++) 
	        {
	    		cartesianTrack_mpc[k] = cartesianTrack_[1 + i + k];
	    	}
	    	

	        if(verbose_) logger_->info("Slide down the desired trajectory\n");

	        i += HMPC_;
	    	robotPublisher.setTerminate(terminate(i, x));

	    }

	    std::this_thread::sleep_for(std::chrono::milliseconds(100));

	    #ifdef DEBUG
	    cnpy::npy_save("../data/state_trajectory_admm_mpc.npy", cartesian_mpc_logger.data(),{1, static_cast<unsigned long>(cartesian_mpc_logger.cols()), static_cast<unsigned long>(cartesian_mpc_logger.rows())}, "w");
		cnpy::npy_save("../data/state_trajectory_admm_mpc_desired.npy", cartesian_desired_logger.data(),{1, static_cast<unsigned long>(cartesian_desired_logger.cols()), static_cast<unsigned long>(cartesian_desired_logger.rows())}, "w");
		#endif
	    std::cout << "Finished the main thread...\n";

		robotPublishThread.detach();
	}
};

#endif



