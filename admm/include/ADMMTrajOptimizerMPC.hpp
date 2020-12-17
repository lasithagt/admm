#ifndef ADMMMPCMAIN_H
#define ADMMMPCMAIN_H

#include <memory>

/* MPC trajectory generation */

#include <iostream>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <string>

#include <Eigen/Dense>


#include "config.h"
#include "ilqrsolver_admm.hpp"
#include "kuka_arm.h"
#include "SoftContactModel.h"
#include "KukaModel.h"
#include "models.h"
#include "admm.hpp"
#include "cost_function_admm.h"
#include "RobotPlant.hpp"


#include "modern_robotics.h"
#include "ik_trajectory.hpp"
#include "ik_solver.hpp"
#include "kuka_robot.hpp"


using namespace std;

class ADMMTrajOptimizerMPC {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ADMMTrajOptimizerMPC();
    ~ADMMTrajOptimizerMPC();

    void run(std::shared_ptr<RobotAbstract>& kukaRobot,  const stateVec_t& init_state, const ContactModel::SoftContactModel& contactModel, const ADMMopt& ADMM_OPTS, const IKTrajectory<IK_FIRST_ORDER>::IKopt& IK_OPT, \
     const Saturation& LIMITS, const std::vector<Eigen::MatrixXd>& cartesianPoses, optimizer::ILQRSolverADMM::traj& result);

    optimizer::ILQRSolverADMM::traj getOptimizerResult(); 

private:
    Eigen::MatrixXd joint_state_traj;
    commandVecTab_t torque_traj;
    stateVecTab_t joint_state_traj_interp;
    commandVecTab_t torque_traj_interp;

protected:
	optimizer::ILQRSolverADMM::traj resultTrajectory;
};

#endif

