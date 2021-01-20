#ifndef ROBOTPLANT_H
#define ROBOTPLANT_H

#include <cstdio>
#include <iostream>
#include <Eigen/Dense>
#include <math.h>

#include "config.h"
#include "models.h"
#include "eigenmvn.hpp"
#include "RobotDynamics.hpp"
#include <mutex>
#include "Plant.hpp"

template<class Dynamics, int S, int C>
class RobotPlant : public Plant<S, C>
{

public:
    enum { StateSize = S, ControlSize = C };
    using Scalar                = double;
    using State                 = typename Plant<S, C>::State;
    using Control               = typename Plant<S, C>::Control;
    using StateNoiseVariance    = Eigen::Matrix<Scalar, stateSize, stateSize>;
    using ControlNoiseVariance  = Eigen::Matrix<Scalar, commandSize, commandSize>;

    using Plant<S, C>::currentState;
    using Plant<S, C>::dt;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RobotPlant(const std::shared_ptr<Dynamics>& robotDynamics, Scalar timeStep, Scalar state_var, Scalar control_var) 
    : Plant<S, C>(timeStep), m_plantDynamics(robotDynamics), sdist_(State::Zero(), state_var * StateNoiseVariance::Identity()),
      cdist_(Control::Zero(), control_var * ControlNoiseVariance::Identity()) {}
        // currentState.setZero();
    

    RobotPlant() = default;
    RobotPlant(const RobotPlant &other) {};
    RobotPlant(RobotPlant &&other) {};

    RobotPlant& operator=(const RobotPlant &other) {};
    RobotPlant& operator=(RobotPlant &&other) {};
    ~RobotPlant() = default;


    /**
     * @brief   Pass information to the Plant in order to apply controls and obtain the new state.
     *
     * The user must provide an implementation that takes in the system state and the control calculated
     * by the optimizer, applies one control to the system, performs any desired calculations or updates,
     * and returns the new state of the system after the application of the control.
     *
     * Note that unlike the Dynamics functor, this function must return a STATE rather than a state transition.
     * This is in the interests of compatibility with real robotic systems whose state estimators will usually
     * return a state rather than a transition.
     *
     * @param x The state calculated by the optimizer for the current time window.
     * @param u The control calculated by the optimizer for the current time window.
     * @return  The new state of the system.
     */
    bool applyControl(const Eigen::Ref<const Control> &u, const Eigen::Ref<const State> &x) 
    {
        std::lock_guard<std::mutex> locker(mu);
        commandVec_t u_noisy = u + 1*cdist_.samples(1);

        State f1 = m_plantDynamics->f(currentState, u_noisy);
        State f2 = m_plantDynamics->f(currentState + 0.5 * dt * f1, u_noisy);
        State f3 = m_plantDynamics->f(currentState + 0.5 * dt * f2, u_noisy);
        State f4 = m_plantDynamics->f(currentState + dt * f3, u_noisy);

        currentState = currentState + (dt/6) * (f1 + 2 * f2 + 2 * f3 + f4) + 0.01*sdist_.samples(1);
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
    std::shared_ptr<Dynamics> m_plantDynamics;
    // State currentState{};

    std::mutex mu;

};
  
#endif // KUKAARM_H
