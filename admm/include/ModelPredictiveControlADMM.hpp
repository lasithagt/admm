#ifndef MPCADMM_H
#define MPCADMM_H

#include <memory>

#include "config.h"
#include "RobotAbstract.h"

#include <iostream>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <string>
#include <chrono>

// concurrency headers
#include <thread>
#include <mutex>
#include <condition_variable>

#include "modern_robotics.h"
#include "differential_ik_trajectory.hpp"
#include "differential_ik_solver.hpp"
#include "cnpy.h"

#include "RobotPublisherMPC.hpp"
#include "logger.hpp"
#include "admm_public.hpp"

/* MPC algorithm wiith compute delay


*/

std::mutex mu_main;
std::condition_variable cv_main;
bool mpcComputeFinished = false;
bool currentStateReceived = false;
bool newControlTrajectorySet = false;
bool init = false;
int NMPC{};

std::once_flag command_start;

std::chrono::time_point<std::chrono::high_resolution_clock> start_, end_;
std::chrono::duration<float, std::milli> elapsed_;

std::chrono::time_point<std::chrono::high_resolution_clock> start_command, end_command;
std::chrono::duration<float, std::milli> elapsed_command;

double delay_compute{0.0};
double delay_network{0.0};

int current_step{0};
int command_steps{0};
int previous_command_steps{0};

template <typename RobotPublisher>
void publishCommands(RobotPublisher& publisher, double dt)
{
	// run until optimizer is publishing
	while (!publisher->terminate) {
		{
			// round up the delay to nearest time step
			std::cout << "\nIn publisher thread..." << std::endl;
			std::cout << "Delay steps: " << previous_command_steps << std::endl;
			double delay_approx =  std::round(delay_compute/10) ; // std::floor(delay_network/10);
			{
				// if mpc comppute is not finished, keep publlishing the command
				{
					std::cout << "Reciving the current state in thread publisher..." <<  std::endl;
					// get current state
					
					// store the states
					publisher->currentState = publisher->getCurrentState();
					current_step            = publisher->getCurrentStep();
					{
						std::unique_lock<std::mutex> lk(mu_main);
						currentStateReceived = true;
						lk.unlock();
						cv_main.notify_one();
						mpcComputeFinished = false;
						std::cout << "notified: state was recived\n" << std::endl;
					}

					// account for delay
					command_steps  = static_cast<int>(delay_approx);
					// std::cout << "DELAY: " << command_steps << std::endl;
					previous_command_steps = 0;
					start_command = std::chrono::high_resolution_clock::now();

					while (mpcComputeFinished==false & command_steps<publisher->getHorizonTimeSteps() & init)
					{
						{
							std::lock_guard<std::mutex> lk(mu_main);
							publisher->publishCommand(command_steps);
							std::cout << "Publishing Control Command..." << command_steps << std::endl;	
						}

						// wait for dt
						std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000) - 1));
						++command_steps;  
						++previous_command_steps;
					}

					end_command     = std::chrono::high_resolution_clock::now();
					elapsed_command = end_command - start_command;
					// std::cout << "DELAY TIME: " << static_cast<double>(elapsed_command.count()) << std::endl;
					// std::cout << "DELAY STEPS: " << previous_command_steps << std::endl;

					{
				    	std::cout << "waiting for the new controls"  <<  std::endl;
				        std::unique_lock<std::mutex> lk(mu_main);
				        auto now = std::chrono::system_clock::now();
				        cv_main.wait_until(lk, now + std::chrono::milliseconds(4), []{return newControlTrajectorySet;});
				        lk.unlock();
				        std::cout << "existing publishing..." << std::endl;
				        newControlTrajectorySet = false;
				    }
				} 
			}
		}
	}

	std::cout << "Finished Publishing Thread" << std::endl;
	return;
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
    using StateGainMatrix   	= commandR_stateC_tab_t;
    using RobotPublisher 	 	= std::shared_ptr<RobotPublisherT>;
    using Result                = OptimizerResultT;            ///< Type of result returned by the optimizer
    using TerminationCondition =
        std::function<bool(int, State)>;     ///< Type of termination condition function

    static const int MPC_BAD_CONTROL_TRAJECTORY = -1;
    Optimizer opt_;
    CostFunction cost_function_;
    bool verbose_;
    Scalar dt_;
    int H_MPC;
    Logger* logger_;
    ControlTrajectory control_trajectory;
    StateTrajectory x_track_;
    std::vector<Eigen::MatrixXd> cartesianTrack_;
    Eigen::MatrixXd cartesian_desired_logger;
    Eigen::MatrixXd cartesian_mpc_logger;
    Eigen::MatrixXd cartesian_mpc_state_logger;

	Eigen::MatrixXd actual_state;
	Eigen::MatrixXd cartesian_actual_state;
	Eigen::MatrixXd cartesian_desired_state;

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
    ModelPredictiveControllerADMM(Scalar dt, int time_steps,  int iterations, bool verbose, Logger *logger, \
	         CostFunction                   &cost_function,
	         Optimizer 						&opt,
			 const TrajectoryDesired<stateSize, NumberofKnotPt> &desiredTrajectory, 
    		 const IKTrajectory<IK_FIRST_ORDER>::IKopt& IK_OPT_) : dt_(dt), H_MPC(time_steps), verbose_(verbose), cost_function_(cost_function),
    		  opt_(opt), IK_OPT(IK_OPT_)
    {
    	x_track_ = desiredTrajectory.stateTrajectory;
    	cartesianTrack_ = desiredTrajectory.cartesianTrajectory;

    	logger_ = logger;
    	control_trajectory.resize(commandSize, H_MPC);
    	cartesian_desired_logger.resize(3, cartesianTrack_.size());
    	cartesian_mpc_logger.resize(3, cartesianTrack_.size());
    	cartesian_mpc_state_logger.resize(3, cartesianTrack_.size());

 
		actual_state.resize(stateSize, cartesianTrack_.size());
		actual_state.setZero();

		cartesian_desired_state.resize(3, cartesianTrack_.size());
		cartesian_desired_state.setZero();

		cartesian_actual_state.resize(3, cartesianTrack_.size());
		cartesian_actual_state.setZero();

    	cartesian_mpc_logger.setZero();

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
	         const Saturation			    &L)
	         // TerminalCostFunction           &terminal_cost_function)
	{

	    if (initial_control_trajectory.cols() != H_MPC)
	    {
	        logger_->error("The size of the control trajectory does not match the number of time steps passed to the optimizer!");
	        std::exit(MPC_BAD_CONTROL_TRAJECTORY);
	    }

	    State x = initial_state, xold = initial_state;
	    Control u;
	    // Scalar true_cost = cost_function.c(xold, initial_control_trajectory[0]);

	    Eigen::MatrixXd x_track_mpc;
	    std::vector<Eigen::MatrixXd> cartesianTrack_mpc;
	    cartesianTrack_mpc.resize(H_MPC + 1);

	    for (int k = 0;k < H_MPC + 1;k++) {
	    	cartesianTrack_mpc[k] = cartesianTrack_[k];
	    }

	    x_track_mpc.resize(stateSize, H_MPC + 1);
	    x_track_mpc = x_track_.block(0, 0, stateSize, H_MPC + 1);


	    scalar_t true_cost = cost_function_->cost_func_expre(0, xold, initial_control_trajectory.col(0), x_track_.col(0));
	    
	    Result result;
	    control_trajectory = initial_control_trajectory;
	    result.uList = initial_control_trajectory;
	    control_trajectory.setZero();

	    u = initial_control_trajectory.col(0);
	    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
	    std::chrono::duration<float, std::milli> elapsed;

	    // get the state strajectory storing vector
	    Eigen::MatrixXd& stateTrajectory = robotPublisher->getStateTrajectory();

    	// to store the state evolution
    	robotPublisher->setInitialState(initial_state);

    	// call the thread
		std::thread robotPublishThread(publishCommands<RobotPublisher>, std::ref(robotPublisher), dt_);

		// start MPC
 	    int64_t i = 0;
 	    int64_t optimizer_iter = 0;
 	    int temp_time{0};

 	    StateTrajectory test;
 	    test.resize(stateSize, static_cast<int>(NumberofKnotPt));
		// start_ = std::chrono::high_resolution_clock::now();

	    while(!terminate(robotPublisher->getCurrentStep(), x))
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
	       	
		    {
		    	std::cout << "waiting for the current state in MPC thread"  <<  std::endl;
		        std::unique_lock<std::mutex> lk(mu_main);
		        cv_main.wait(lk, []{return currentStateReceived;});

		        xold = robotPublisher->currentState;
		        lk.unlock();
		       	currentStateReceived = false;

		       	i = current_step;
		       	std::cout << "\nX_current" << xold.transpose() << std::endl;

		       	std::cout << "\nCurrent step :" << robotPublisher->getCurrentStep() << " " << std::endl;
		    }

	        /* Slide down the control trajectory */
	        // control_trajectory.leftCols(H_ - 1) = result.uList.rightCols(H_ - 1);
	        // control_trajectory = result.uList;
	        if (optimizer_iter > 1)
	        {
	        	if(verbose_) logger_->info("Slide down the control trajectory\n");
		        control_trajectory = initial_control_trajectory;

		       	// slide down the control for state and cartesian state
		       	if(verbose_) logger_->info("Slide down the desired trajectory\n");

		       	auto H_TRACK = H_MPC + 1;
		       	if (i + H_TRACK > static_cast<int>(NumberofKnotPt) + 1) {H_TRACK = static_cast<int>(NumberofKnotPt) - i;}

		       	actual_cartesian_pose = mr::FKinSpace(IK_OPT.M, IK_OPT.Slist,xold.head(7));
		       	cartesianTrack_mpc[0] = actual_cartesian_pose;
				cartesian_actual_state.col(optimizer_iter) = actual_cartesian_pose.col(3).head(3);


		        for (int k = 1;k < H_TRACK;k++) 
		        {	
		        	cartesianTrack_mpc[k] = cartesianTrack_[i + k];
		    	}
				cartesian_desired_state.col(optimizer_iter) = cartesianTrack_mpc.at(1).col(3).head(3);
		    }

	        // Run the optimizer to obtain the next control trajectory
	        {	
		        opt_.solve(xold, control_trajectory, x_track_mpc, cartesianTrack_mpc, rho, L);
		        ++optimizer_iter;
		        std::cout << "Optimizer Iteration: " << optimizer_iter << std::endl;
	    	}

	        // Apply the control to the plant and obtain the new state
	        x = xold; 
		    {
		        std::lock_guard<std::mutex> lk(mu_main);
		        mpcComputeFinished = true;
		        std::cout << "MPC compute finished...\n";
		    }

	        result = opt_.getLastSolvedTrajectory();

        	// apply to the plant. call a child thread 
        	// set the control trajectory
        	{
        		std::unique_lock<std::mutex> lk(mu_main);
        		control_trajectory = result.uList;
        		std::cout << "set publisher controls" << std::endl;
        		robotPublisher->setControlBuffer(control_trajectory);
				newControlTrajectorySet = true;
		
				lk.unlock();
				end = std::chrono::high_resolution_clock::now();
			    elapsed = end - start;

		        delay_compute = (optimizer_iter == 1) ? 0.0 : previous_command_steps * 10 + 10; // + 200; // + 100;
		        
		        std::cout << "DELAY COMPUTE: " << delay_compute << std::endl;

		       
				// start publishing commands
				if (optimizer_iter == 1) {init = true;};
				robotPublisher->terminate = terminate(robotPublisher->getCurrentStep(), x);
				cv_main.notify_one();
        	}
        	
        	
        	robotPublisher->setOptimizerStatesGains(result.xList, std::move(result.KList), result.kList);

	        if(verbose_)
	        {
	            logger_->info("Received new state from plant: ");
	            for(int n = 0; n < x.rows(); ++n) { logger_->info("%f ", x(n)); }
	            logger_->info("\n");
	        }

	        // Calculate the true cost for this time step
	        true_cost = cost_function_->cost_func_expre(0, xold, u, x_track_.col(i));

	        if(verbose_) logger_->info("True cost for time step %d: %f\n", i, true_cost);	
	  

	    }

	    robotPublishThread.join();


	    // save data
        #ifdef DEBUG

        for (int i = 0; i < static_cast<int>(NumberofKnotPt)+1; i++) 
        {
	    	cartesian_desired_logger.col(i) = cartesianTrack_.at(i).col(3).head(3);
			actual_cartesian_pose = mr::FKinSpace(IK_OPT.M, IK_OPT.Slist, stateTrajectory.col(i).head(7));
			cartesian_mpc_logger.col(i) = actual_cartesian_pose.col(3).head(3);

		}

        for (int i = 0; i < static_cast<int>(static_cast<int>(H_MPC) + 1); i++) 
        {

			actual_cartesian_pose = mr::FKinSpace(IK_OPT.M, IK_OPT.Slist, result.xList.col(i).head(7));
			cartesian_mpc_state_logger.col(i) = actual_cartesian_pose.col(3).head(3);

		}
	    #endif

	    #ifdef DEBUG
	    logger_->info("Saving Data...\n");
	    cnpy::npy_save("/home/lasitha/Documents/Github/bullet/examples/KUKAEnv/ddp_contact/standalone/data/state_trajectory_admm_mpc.npy", cartesian_mpc_logger.data(),{1, static_cast<unsigned long>(cartesian_mpc_logger.cols()), static_cast<unsigned long>(cartesian_mpc_logger.rows())}, "w");
		
		cnpy::npy_save("/home/lasitha/Documents/Github/bullet/examples/KUKAEnv/ddp_contact/standalone/data/state_trajectory_admm_state_mpc.npy", cartesian_mpc_state_logger.data(),{1, static_cast<unsigned long>(cartesian_mpc_state_logger.cols()), static_cast<unsigned long>(cartesian_mpc_state_logger.rows())}, "w");
		cnpy::npy_save("/home/lasitha/Documents/Github/bullet/examples/KUKAEnv/ddp_contact/standalone/data/state_trajectory_admm_mpc_desired.npy", cartesian_desired_logger.data(),{1, static_cast<unsigned long>(cartesian_desired_logger.cols()), static_cast<unsigned long>(cartesian_desired_logger.rows())}, "w");
		
		cnpy::npy_save("/home/lasitha/Documents/Github/bullet/examples/KUKAEnv/ddp_contact/standalone/data/state_trajectory_actual_state.npy", cartesian_actual_state.data(),{1, static_cast<unsigned long>(cartesian_actual_state.cols()), static_cast<unsigned long>(cartesian_actual_state.rows())}, "w");
		cnpy::npy_save("/home/lasitha/Documents/Github/bullet/examples/KUKAEnv/ddp_contact/standalone/data/state_trajectory_cartesian_desired.npy", cartesian_desired_state.data(),{1, static_cast<unsigned long>(cartesian_desired_state.cols()), static_cast<unsigned long>(cartesian_desired_state.rows())}, "w");

		logger_->info("Saveds Data...\n");

		#endif

		logger_->info("Finished the main thread...\n");

		
	}
};

#endif



