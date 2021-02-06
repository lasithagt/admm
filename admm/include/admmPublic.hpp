#ifndef ADMMPUBLIC_H
#define ADMMPUBLIC_H  


  // data structure for saturation limits
  struct Saturation {
    Saturation() {}

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
    // parameters for ADMM, penelty terms
    int ADMMiterMax;

  };


  // // data structure for admm options
  struct TrajectoryTrack 
  {
    TrajectoryTrack(double des_normal_force) {}
    const std::vector<Eigen::MatrixXd> cartesianTrajectory;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> stateTrajectory;

  };


  // data structure for admm options
  struct ADMM_MPCopt {
    ADMM_MPCopt() = default;
    ADMM_MPCopt(const ADMMopt& admm_opt, const Saturation& limits, double dt, unsigned int horizon=100) : admm_opt_(admm_opt), limits_(limits), horizon_(horizon), dt_(dt) {}

    ADMMopt admm_opt_;
    Saturation limits_;
    unsigned int horizon_;
    double dt_;
    
  };


    // data structure for admm options
  struct ADMM_MPCconfig {
    ADMM_MPCconfig() = default;
    ADMM_MPCconfig(const ADMM_MPCopt& admm_mpc_opt, const IKTrajectory<IK_FIRST_ORDER>::IKopt& ik_opt) : admm_mpc_opt_(admm_mpc_opt), ik_opt_(ik_opt) {}

    ADMM_MPCopt admm_mpc_opt_;
    IKTrajectory<IK_FIRST_ORDER>::IKopt ik_opt_;
  };




  #endif