#ifndef PLANT_H
#define PLANT_H

#include <cstdio>
#include <iostream>
#include <Eigen/Dense>
#include <math.h>

#include "config.h"
#include "models.h"
#include "eigenmvn.hpp"
#include "kuka_arm.h"

template<class Dynamics_, int S, int C>
class Plant
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
    Plant(Dynamics& RobotDynamics, Scalar dt_, Scalar state_var, Scalar control_var)
    : m_plantDynamics(RobotDynamics), dt(dt_), sdist_(State::Zero(), state_var * StateNoiseVariance::Identity()),
      cdist_(Control::Zero(), control_var * ControlNoiseVariance::Identity()) {

        // robotPublisher = RobotPublisherMPC();

    }

    Plant() = default;
    Plant(const Plant &other) = default;
    Plant(Plant &&other) = default;

    Plant& operator=(const Plant &other) = default;
    Plant& operator=(Plant &&other) = default;
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
    virtual bool applyControl(const Eigen::Ref<const Control> &u) = 0;


    bool setInitialState(const Eigen::Ref<const State> &x)
    {
        currentState = x;
        return true;
    }

    State getCurrentState()
    {
        return currentState;
    }
    
private:
    Dynamics& m_plantDynamics;
    Scalar dt;
    Eigen::EigenMultivariateNormal<double, StateSize> sdist_;
    Eigen::EigenMultivariateNormal<double, ControlSize> cdist_;

    State currentState;

};
  
#endif // KUKAARM_H
