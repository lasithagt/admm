#ifndef ADMM_H
#define ADMM_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <string>

#include <Eigen/Dense>

#include "config.h"
#include "spline.h"
#include "ilqrsolver_admm.hpp"
#include "kuka_arm.h"
#include "SoftContactModel.h"
#include "KukaModel.h"
#include "models.h"
#include "cost_function_admm.h"

#include "modern_robotics.h"
#include "ik_trajectory.hpp"
#include "ik_solver.hpp"
#include "kuka_robot.hpp"

#include "curvature.hpp"


/* ADMM trajectory generation */
class ADMM {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  // data structure for saturation limits
  struct Saturation {
    Saturation() {

    }

    Eigen::Matrix<double, 2, stateSize> stateLimits;
    Eigen::Matrix<double, 2, commandSize> controlLimits;
  };


  // data structure for admm options
  struct ADMMopt {
    ADMMopt(double dt_, double tolFun_, double tolGrad_, unsigned int iterMax_, 
      int ADMMiterMax_) : dt(dt_), tolFun(tolFun_), tolGrad(tolGrad_), iterMax(iterMax_), ADMMiterMax(ADMMiterMax_) {}
    double dt;
    double tolFun; // 1e-5; // relaxing default value: 1e-10; - reduction exit crieria
    double tolGrad; // relaxing default value: 1e-10; - gradient exit criteria
    unsigned int iterMax; //DDP iteration max
    // parameters for ADMM, penelty terms
    int ADMMiterMax;
  };
  
  ADMM(const ADMMopt& ADMM_opt, const IKTrajectory<IK_FIRST_ORDER>::IKopt& IK_opt);

  // template<class T>
  void run(std::shared_ptr<KUKAModelKDL>& kukaRobot, KukaArm& KukaArmModel, const stateVec_t& xinit, const stateVecTab_t& xtrack, 
    const std::vector<Eigen::MatrixXd>& cartesianTrack, const Eigen::VectorXd& rho, const Saturation& L);

  Eigen::MatrixXd projection(const stateVecTab_t& xnew, const Eigen::MatrixXd& cnew, const commandVecTab_t& unew, const Saturation& L);
  void contact_update(std::shared_ptr<KUKAModelKDL>& kukaRobot, const stateVecTab_t& xnew, Eigen::MatrixXd* cnew);


protected:
  models::KUKA robotIK;

  Curvature curve;
  Saturation projectionLimits;
  ADMMopt ADMM_OPTS;
  IKTrajectory<IK_FIRST_ORDER>::IKopt IK_OPT;

  optimizer::ILQRSolverADMM::traj lastTraj;
  stateVecTab_t joint_state_traj;
  commandVecTab_t torque_traj;
  stateVecTab_t joint_state_traj_interp;
  commandVecTab_t torque_traj_interp;

  unsigned int N;

  /* Initalize Primal and Dual variables */

  // primal parameters
  stateVecTab_t xnew;
  Eigen::MatrixXd qnew;
  Eigen::MatrixXd cnew;
  commandVecTab_t unew;

  // stateVecTab_t x_avg;
  Eigen::MatrixXd q_avg;
  stateVecTab_t x_lambda_avg;

  stateVecTab_t xbar;
  Eigen::MatrixXd cbar;
  commandVecTab_t ubar;
  Eigen::MatrixXd qbar;

  stateVecTab_t xbar_old; // "old" for last ADMM iteration 
  Eigen::MatrixXd cbar_old;
  commandVecTab_t ubar_old; 
  
  // dual parameters
  stateVecTab_t x_lambda;
  Eigen::MatrixXd c_lambda;
  commandVecTab_t u_lambda;
  Eigen::MatrixXd q_lambda;

  stateVecTab_t x_temp;
  Eigen::MatrixXd c_temp;
  commandVecTab_t u_temp;
  Eigen::MatrixXd q_temp;

  commandVecTab_t u_0;;

  Eigen::MatrixXd xubar; // for projection

  // primal residual
  std::vector<double> res_x;
  std::vector<double> res_q;
  std::vector<double> res_u;
  std::vector<double> res_c;

  // dual residual
  std::vector<double> res_xlambda;
  std::vector<double> res_qlambda;
  std::vector<double> res_ulambda;
  std::vector<double> res_clambda;

  std::vector<double> final_cost;

  // joint_positions_IK
  Eigen::MatrixXd joint_positions_IK;

  Eigen::VectorXd L;
  Eigen::VectorXd R_c;
  Eigen::MatrixXd k;

  Eigen::MatrixXd X_curve;

  IKTrajectory<IK_FIRST_ORDER> IK_solve;

};

#endif



