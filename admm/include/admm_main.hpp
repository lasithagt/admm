#ifndef ADMM_MAIN_H
#define ADMM_MAIN_H

#include <memory>
#include <Eigen/Dense>


#include "admm.hpp"
#include "SoftContactModel.h"


class FULL_ADMM {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FULL_ADMM(unsigned int N_, double dt_);
    ~FULL_ADMM();

    void run(std::shared_ptr<RobotAbstract>& kukaRobot, stateVec_t init_state, optimizer::ILQRSolverADMM::OptSet& solverOptions, ADMM::ADMMopt& ADMM_OPTS, IKTrajectory<IK_FIRST_ORDER>::IKopt& IK_OPT, ADMM::Saturation& LIMITS, ContactModel::ContactParams& cp, std::vector<Eigen::MatrixXd>& cartesianPoses); 


    optimizer::ILQRSolverADMM::traj getOptimizerResult(); 


protected:
    unsigned int N;
    double dt;
    optimizer::ILQRSolverADMM::traj resultTrajectory;
};

#endif  //ROBOT_ABSTRACT_H
