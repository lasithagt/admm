#ifndef ADMMMPCMAIN_H
#define ADMMMPCMAIN_H

#include <memory>

/* MPC trajectory generation */

#include <iostream>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <string>


#include "ADMMMultiBlock.hpp"
#include "RobotPlant.hpp"

#include "modern_robotics.h"
#include "DiffIKTrajectory.hpp"
#include "DiffIKSolver.hpp"


using namespace std;

class ADMMTrajOptimizerMPC {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ADMMTrajOptimizerMPC();
    ~ADMMTrajOptimizerMPC();

    void run(const std::shared_ptr<RobotAbstract>& kukaRobot,  const stateVec_t& init_state, const ContactModel::SoftContactModel<double>& contactModel, const ADMMopt& ADMM_OPTS, const IKTrajectory<IK_FIRST_ORDER>::IKopt& IK_OPT, \
     const Saturation& LIMITS, const std::vector<Eigen::MatrixXd>& cartesianPoses, optimizer::IterativeLinearQuadraticRegulatorADMM::traj& result);

    optimizer::IterativeLinearQuadraticRegulatorADMM::traj getOptimizerResult(); 

protected:
	optimizer::IterativeLinearQuadraticRegulatorADMM::traj resultTrajectory;
    Eigen::MatrixXd joint_state_traj;

};

#endif

