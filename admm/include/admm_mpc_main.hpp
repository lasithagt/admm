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
#include "mpc_admm.hpp"
#include "plant.h"


#include "modern_robotics.h"
#include "ik_trajectory.hpp"
#include "ik_solver.hpp"
#include "kuka_robot.hpp"


using namespace std;

class MPC_ADMM {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MPC_ADMM();
    ~MPC_ADMM();

    void run(std::shared_ptr<RobotAbstract>& kukaRobot, ContactModel::SoftContactModel& contactModel, const stateVec_t& xinit, const stateVecTab_t& xtrack, std::vector<Eigen::MatrixXd> &cartesianPoses, optimizer::ILQRSolverADMM::traj& result);
private:
    Eigen::MatrixXd joint_state_traj;
    commandVecTab_t torque_traj;
    stateVecTab_t joint_state_traj_interp;
    commandVecTab_t torque_traj_interp;

protected:
    optimizer::ILQRSolverADMM::traj lastTraj;
};

#endif

