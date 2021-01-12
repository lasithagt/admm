#ifndef ADMM_TRAJ_H
#define ADMM_TRAJ_H

#include <memory>
#include <Eigen/Dense>

#include "ADMMMultiBlock.hpp"


class ADMMTrajOptimizer {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ADMMTrajOptimizer(unsigned int N_, double dt_);
    ~ADMMTrajOptimizer();

    void run(const std::shared_ptr<RobotAbstract>& kukaRobot, stateVec_t init_state, optimizer::IterativeLinearQuadraticRegulatorADMM::OptSet& solverOptions, ADMMopt& ADMM_OPTS, IKTrajectory<IK_FIRST_ORDER>::IKopt& IK_OPT, \
    	Saturation& LIMITS, ContactModel::ContactParams<double>& cp, std::vector<Eigen::MatrixXd>& cartesianPoses); 
    optimizer::IterativeLinearQuadraticRegulatorADMM::traj getOptimizerResult(); 


private:
    unsigned int N;
    double dt;
    optimizer::IterativeLinearQuadraticRegulatorADMM::traj resultTrajectory;
};

#endif  //ROBOT_ABSTRACT_H
