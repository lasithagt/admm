#ifndef PLANT_H
#define PLANT_H

// from standard C++
#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>         // std::thread
#include <math.h>
#include <mutex>
#include <condition_variable>


// Eigen libary
#include <Eigen/Dense>

// headers in this project
#include "config.h"
#include "models.h"
#include "RobotDynamics.hpp"


// using namespace std::literals::chrono_literals;


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
template<typename Plant, int S, int C>
class RobotPublisherMPC
{
    enum { StateSize = S, ControlSize = C };
    using State             = stateVec_t;
    using Scalar            = double;
    using Control           = commandVec_t;
    using StateTrajectory   = stateVecTab_t;
    using ControlTrajectory = commandVecTab_t;

public:
    RobotPublisherMPC(Plant& robotPlant, int N, Scalar dt_)
    : m_robotPlant(robotPlant), dt(dt_), N_commands(N) {

        controlBuffer.resize(C, N_commands);
        controlBuffer.setZero();

        increment = 0;
    }

    RobotPublisherMPC() = default;
    RobotPublisherMPC(const RobotPublisherMPC &other) = default;
    RobotPublisherMPC(RobotPublisherMPC &&other) = default;

    RobotPublisherMPC& operator=(const RobotPublisherMPC &other) {

    }

    RobotPublisherMPC& operator=(RobotPublisherMPC &&other) = default;
    ~RobotPublisherMPC() = default;

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
    bool publishCommands()
    {
      
        // Wait until main() sends data
        // std::unique_lock<std::mutex> lk(mu);
        // cv.wait(lk, []{return false;});

        // run until optimizer is publishing
        // while (!terminate) {
          // std::lock_guard<std::mutex> locker(mu);
          // cv.wait(locker, [this]{return optimizerFinished;});

          while (increment!=N_commands) {
          // for (int i = 0;i<N_commands;i++) {
            
            u_scratch  = controlBuffer.col(increment);

            m_robotPlant.applyControl(u_scratch);

            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
              // }

            // get current state
            currentState = m_robotPlant.getCurrentState();
            // firstStateReceived = true;

            // store the states
            stateBuffer->col(increment) = currentState;   
            std::cout << "Publishing Control Command..." << std::endl;

            increment++;
            optimizerFinished = false;

          }

          // wait for dt
          // auto t = dt * 1000;
          // std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // }

        return true;
    }



    // set the control buffer from MPC optimizer
    bool setControlBuffer(const Eigen::Ref<const Eigen::MatrixXd> &u)
    {
        // std::lock_guard<std::mutex> locker(mu);
        controlBuffer = u;
        return true;
    }

    // set the control buffer from MPC optimizer
    bool setStateBuffer(Eigen::MatrixXd* x)
    {
        stateBuffer = x;
        return true;
    }

    bool setCurrentState(const Eigen::Ref<const State> &x)
    {
        currentState = x;
        return true;
    }

    State getCurrentState()
    {
        return currentState;
    }

    void setTerminate(bool t) {
        terminate = t;
    }

    void setInitialState(const Eigen::Ref<const State> xinit) {
        std::lock_guard<std::mutex> locker(mu);
        m_robotPlant.setInitialState(xinit);
        stateBuffer->col(0) = xinit;
    }

    void reset() {
      increment = 0;
      // optimizerFinished = 1;
    }

    void set() {
      increment = 1;
    }


    
public:
    // storing variable
    commandVec_t u_scratch;
    stateVec_t  x_scratch;

    Plant m_robotPlant;
    Scalar dt;

    // length of the MPC trajectory commands
    int N_commands;

    // store command vector
    Eigen::MatrixXd controlBuffer;

    // store the states in buffer
    Eigen::MatrixXd* stateBuffer;

    // current state of the robot
    State currentState;

    std::mutex mu;
    std::condition_variable cv;

    bool terminate;

    bool optimizerFinished;

    int increment;

};
  
#endif // KUKAARM_H
