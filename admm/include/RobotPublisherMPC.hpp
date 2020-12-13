#ifndef PLANT_H
#define PLANT_H

// from standard C++
#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>         // std::thread
#include <math.h>
#include <mutex>

// Eigen libary
#include <Eigen/Dense>

// headers in this project
#include "config.h"
#include "models.h"
#include "kuka_arm.h"


using namespace std::literals::chrono_literals;


/**
 * @brief   Control layer to publish commands in MPC
 *
 * The user must provide an implementation that takes in the system state and the control calculated
 * by the
  optimizer, applies one control to the system, performs any desired calculations or updates,
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
template<class Plant, int S, int C>
class RobotPublisherMPC
{
    enum { StateSize = S, ControlSize = C };
    using State             = stateVec_t;
    using Scalar            = double;
    using Control           = commandVec_t;
    using StateTrajectory   = stateVecTab_t;
    using ControlTrajectory = commandVecTab_t;

public:
    RobotPublisherMPC(Plant& robotPlant, Scalar dt_)
    : m_robotPlant(robotPlant), dt(dt_) {}

    RobotPublisherMPC() = default;
    RobotPublisherMPC(const RobotPublisherMPC &other) = default;
    RobotPublisherMPC(RobotPublisherMPC &&other) = default;

    RobotPublisherMPC& operator=(const RobotPublisherMPC &other) = default;
    RobotPublisherMPC& operator=(RobotPublisherMPC &&other) = default;
    ~RobotPublisherMPC() = default;



    bool publishCommands() {
        m_plantDynamics.set
    }


    // return the publisher thread
    std::thread publisherThread() {
        return std::thread([=] { publishCommands(); });
    }

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
    bool publishCommands(const Eigen::Ref<const Control> &u)
    {
        for (int i=0;i<N_commands;i++) {

            u_scratch  = u.col(i) + cdist_.samples(1);
            // wait for dt
            m_robotPlant.applyControl(u_scratch);

            // get current state
            currentState = m_robotPlant.getCurrentState();
        }

        return true;
    }


    // set the control buffer from MPC optimizer
    bool setControlBuffer(const Eigen::Ref<const Matrix<double, ,>> &u)
    {
        controlBuffer = u;
        return true;
    }


    bool setCurrentState(const Eigen::Ref<const State> &x)
    {
        currentState = x;
        return true;
    }


    
private:
    // storing variable
    commandVec_t u_scratch;
    stateVec_t  x_scratch;

    Plant& m_robotPlant;
    Scalar dt;

    // length of the MPC trajectory commands
    unsigned int N_commands;

    // store command vector
    Eigen::Matrix<double, ,> controlBuffer;

    // current state of the robot
    State currentState;

};
  
#endif // KUKAARM_H
