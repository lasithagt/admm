#ifndef ADMM_MAIN_H
#define ADMM_MAIN_H

#include <memory>
#include <Eigen/Dense>


#include "admm.hpp"


class FULL_ADMM {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FULL_ADMM(unsigned int N_, double dt_);
    ~FULL_ADMM();

    void run(std::shared_ptr<RobotAbstract>& kukaRobot, optimizer::ILQRSolverADMM::OptSet& solverOptions, ADMM::ADMMopt& ADMM_OPTS, IKTrajectory<IK_FIRST_ORDER>::IKopt& IK_OPT);


protected:
    unsigned int N;
    double dt;
};

#endif  //ROBOT_ABSTRACT_H
