#ifndef ADMMPUBLIC_H
#define ADMMPUBLIC_H  

#include "differential_ik_trajectory.hpp"


  // data structure for saturation limits
  struct Saturation 
  {
    Saturation() = default;

    Eigen::Matrix<double, 2, stateSize> stateLimits;
    Eigen::Matrix<double, 2, commandSize> controlLimits;
  };


  // data structure for admm options
  struct ADMMopt {
    ADMMopt() = default;
    ADMMopt(double dt_, double tolFun_, double tolGrad_, unsigned int iterMax_, 
      int ADMMiterMax_) : dt(dt_), tolFun(tolFun_), tolGrad(tolGrad_), iterMax(iterMax_), ADMMiterMax(ADMMiterMax_) {}
    double dt;
    double tolFun; 
    double tolGrad; 
    unsigned int iterMax; // DDP iteration max
    int ADMMiterMax;

  };


  // desired trajectory to track
  template<int StateSize, uint Horizon>
  struct TrajectoryDesired 
  {
    TrajectoryDesired() 
    {
      cartesianTrajectory.resize(static_cast<int>(Horizon) + 1);
      stateTrajectory.resize(StateSize, static_cast<int>(Horizon) + 1);
      stateTrajectory.setZero();
    }

    std::vector<Eigen::MatrixXd> cartesianTrajectory;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> stateTrajectory;

  };


  struct ADMM_MPCopt 
  {
    ADMM_MPCopt() = default;
    ADMM_MPCopt(const ADMMopt& admm_opt, const Saturation& limits) : ADMM_opt_(admm_opt), limits_(limits) {}
    ADMMopt ADMM_opt_;
    Saturation limits_;
    
  };


  // TODO: pass rho as a parameter
  struct ADMM_MPCconfig 
  {
    ADMM_MPCconfig() = default;
    ADMM_MPCconfig(const ADMM_MPCopt& admm_mpc_opt, const IKTrajectory<IK_FIRST_ORDER>::IKopt& ik_opt, double dt, unsigned int horizon=100) : ADMM_MPC_opt_(admm_mpc_opt), IK_opt_(ik_opt)
    , horizon_(horizon), dt_(dt) 
    {
      rho_.resize(5);
      rho_ << 50, 0.1, 0.00001, 0, 2;
    }

    ADMM_MPCopt ADMM_MPC_opt_;
    IKTrajectory<IK_FIRST_ORDER>::IKopt IK_opt_;

    Eigen::VectorXd rho_;
    unsigned int horizon_;
    double dt_;
  };




  #endif