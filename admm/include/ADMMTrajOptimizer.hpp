#ifndef ADMM_TRAJ_H
#define ADMM_TRAJ_H

#include <memory>
#include <Eigen/Dense>

#include "ADMMMultiBlock.hpp"
#include "SoftContactModel.h"


class ADMMTrajOptimizer {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ADMMTrajOptimizer(unsigned int N_, double dt_);
    ~ADMMTrajOptimizer();

    void run(std::shared_ptr<RobotAbstract>& kukaRobot, stateVec_t init_state, optimizer::ILQRSolverADMM::OptSet& solverOptions, ADMMopt& ADMM_OPTS, IKTrajectory<IK_FIRST_ORDER>::IKopt& IK_OPT, \
    	Saturation& LIMITS, ContactModel::ContactParams& cp, std::vector<Eigen::MatrixXd>& cartesianPoses); 
    optimizer::ILQRSolverADMM::traj getOptimizerResult(); 


private:
    unsigned int N;
    double dt;
    optimizer::ILQRSolverADMM::traj resultTrajectory;
};

#endif  //ROBOT_ABSTRACT_H
