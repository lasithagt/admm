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
#include "plant.hpp"

#include "modern_robotics.h"
#include "differential_ik_trajectory.hpp"
#include "differential_ik_solver.hpp"
#include "robot_dynamics.hpp"
#include "RobotPublisherMPC.hpp"
#include "admm_public.hpp"


// using namespace std;
// template<>
class ADMMTrajOptimizerMPC {

public:
	using State 		 = Eigen::Matrix<double, stateSize, 1>;
    using Dynamics       = admm::Dynamics<RobotAbstract, stateSize, commandSize>;
	using PlantPublisher = RobotPublisherMPC<Plant<Dynamics, stateSize, commandSize>, stateSize, commandSize>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ADMMTrajOptimizerMPC();
    ~ADMMTrajOptimizerMPC();

    void run(const std::shared_ptr<RobotAbstract>& kukaRobot,  std::shared_ptr<PlantPublisher>& plant, const State& init_state, const ContactModel::SoftContactModel<double>& contactModel,
        const ADMM_MPCconfig& admmMPC_config, const TrajectoryDesired<stateSize, NumberofKnotPt>& desiredTrajectory, optimizer::IterativeLinearQuadraticRegulatorADMM::traj& result);

    optimizer::IterativeLinearQuadraticRegulatorADMM::traj getOptimizerResult(); 

protected:
	optimizer::IterativeLinearQuadraticRegulatorADMM::traj resultTrajectory;
    Eigen::MatrixXd joint_state_traj;

};

#endif

