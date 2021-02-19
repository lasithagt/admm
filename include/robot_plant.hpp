#ifndef ROBOTPLANT_H
#define ROBOTPLANT_H

#include <cstdio>
#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <mutex>

#include "config.h"
#include "eigenmvn.hpp"
#include "plant.hpp"

template<class Dynamics, int S, int C>
class RobotPlant : public Plant<Dynamics, S, C> {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum { StateSize = S, ControlSize = C };
    using Scalar                = double;
    using State                 = typename Plant<Dynamics, S, C>::State;
    using Control               = typename Plant<Dynamics, S, C>::Control;
    using StateNoiseVariance    = Eigen::Matrix<Scalar, stateSize, stateSize>;
    using ControlNoiseVariance  = Eigen::Matrix<Scalar, commandSize, commandSize>;

    using Plant<Dynamics, S, C>::currentState;
    using Plant<Dynamics, S, C>::dt;


    RobotPlant(const std::shared_ptr<Dynamics>& robotDynamics, Scalar timeStep, Scalar state_var, Scalar control_var) 
    : Plant<Dynamics, S, C>(robotDynamics, timeStep), sdist_(State::Zero(), state_var * StateNoiseVariance::Identity()),
      cdist_(Control::Zero(), control_var * ControlNoiseVariance::Identity()) {}
    

    RobotPlant() = default;
    RobotPlant(const RobotPlant &other) {};
    RobotPlant(RobotPlant &&other) {};

    RobotPlant& operator=(const RobotPlant &other) {};
    RobotPlant& operator=(RobotPlant &&other) {};
    ~RobotPlant() = default;

    bool applyControl(const Eigen::Ref<const Control> &u, const Eigen::Ref<const State> &x) 
    {
        std::lock_guard<std::mutex> locker(mu);
        Control u_noisy = u + 0*cdist_.samples(1);

        State f1 = this->m_plantDynamics->f(currentState, u_noisy);
        State f2 = this->m_plantDynamics->f(currentState + 0.5 * dt * f1, u_noisy);
        State f3 = this->m_plantDynamics->f(currentState + 0.5 * dt * f2, u_noisy);
        State f4 = this->m_plantDynamics->f(currentState + dt * f3, u_noisy);

        currentState = currentState + (dt/6) * (f1 + 2 * f2 + 2 * f3 + f4) + 0.0*sdist_.samples(1);
        return true;
    }

    bool setInitialState(const Eigen::Ref<const State> &x) 
    {
        currentState = x;
        return true;
    }

    const State& getCurrentState() 
    {   
        std::lock_guard<std::mutex> locker(mu);
        return currentState;
    }
    
private:
    // Scalar dt;
    Eigen::EigenMultivariateNormal<double, StateSize> sdist_;
    Eigen::EigenMultivariateNormal<double, ControlSize> cdist_;

    std::mutex mu;

};
  
#endif // KUKAARM_H
