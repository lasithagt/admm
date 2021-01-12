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

template<class Dynamics_, int S, int C>
class RobotPlant
{
    enum { StateSize = S, ControlSize = C };
    using State             = stateVec_t;
    using Scalar            = double;
    using Control           = commandVec_t;
    using StateTrajectory   = stateVecTab_t;
    using ControlTrajectory = commandVecTab_t;
    using Dynamics          = Dynamics_;
    using StateNoiseVariance    = Eigen::Matrix<Scalar, stateSize, stateSize>;
    using ControlNoiseVariance  = Eigen::Matrix<Scalar, commandSize, commandSize>;
    // using Eigen::internal;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RobotPlant(Dynamics& kukaRobot, Scalar dt_, Scalar state_var, Scalar control_var)
    : dt(dt_), sdist_(State::Zero(), state_var * StateNoiseVariance::Identity()),
      cdist_(Control::Zero(), control_var * ControlNoiseVariance::Identity()) {
        m_plantDynamics = &kukaRobot;
        currentState.setZero();

    }

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
    {   std::lock_guard<std::mutex> locker(mu);
        return currentState;
    }
    
private:
    Dynamics* m_plantDynamics;
    Scalar dt;
    Eigen::EigenMultivariateNormal<double, StateSize> sdist_;
    Eigen::EigenMultivariateNormal<double, ControlSize> cdist_;

    State currentState{};

    std::mutex mu;

};
  
#endif // KUKAARM_H
