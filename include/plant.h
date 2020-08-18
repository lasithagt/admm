#ifndef PLANT_H
#define PLANT_H

#include <cstdio>
#include <iostream>
#include <Eigen/Dense>
#include <math.h>

#include "config.h"
// #include "KukaModel.h"
#include "models.h"
#include "eigenmvn.hpp"
#include "kuka_arm.h"

template<class Dynamics_, int S, int C>
class KukaPlant
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
    KukaPlant(Dynamics& kukaRobot, Scalar dt, Scalar state_var, Scalar control_var)
    : kukaRobot_(kukaRobot), dt_(dt), sdist_(State::Zero(), state_var * StateNoiseVariance::Identity()),
      cdist_(Control::Zero(), control_var * ControlNoiseVariance::Identity()) {}

    KukaPlant() = default;
    KukaPlant(const KukaPlant &other) = default;
    KukaPlant(KukaPlant &&other) = default;
    KukaPlant& operator=(const KukaPlant &other) = default;
    KukaPlant& operator=(KukaPlant &&other) = default;
    ~KukaPlant() = default;


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
    State f(const Eigen::Ref<const State> &x, const Eigen::Ref<const Control> &u)
    {
        commandVec_t u_noisy = u + cdist_.samples(1);

        stateVec_t xnew = x + kukaRobot_.kuka_arm_dynamics(x, u_noisy) * dt_ + sdist_.samples(1);
        // Eigen::Map<State>(sx.data(), StateSize) = xnew;
        // Eigen::Map<Control>(su.data(), ControlSize) = u_noisy;
        return xnew;
    }
    
private:
    Dynamics& kukaRobot_;
    Scalar dt_;
    Eigen::EigenMultivariateNormal<double, StateSize> sdist_;
    Eigen::EigenMultivariateNormal<double, ControlSize> cdist_;
};
  
#endif // KUKAARM_H
