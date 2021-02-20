#ifndef ROBOT_PUBLISHER_MPC_H
#define ROBOT_PUBLISHER_MPC_H

// from standard C++
#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>       
#include <math.h>
#include <mutex>
#include <condition_variable>

// headers in this project
#include "config.h"


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
protected:
    enum { StateSize = S, ControlSize = C };
    using State             = stateVec_t;
    using Scalar            = double;
    using Control           = commandVec_t;
    using StateTrajectory   = stateVecTab_t;
    using ControlTrajectory = commandVecTab_t;
    using StateGainMatrix   = commandR_stateC_tab_t;

    std::shared_ptr<Plant> m_robotPlant{};
    int command_step{0};
    int N_commands{}; // length of the MPC trajectory commands
    int N_Trajectory{};
    Control u{};

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // storing variable
    commandVec_t u_scratch;
    stateVec_t  x_scratch;
    Scalar dt;

    Eigen::MatrixXd controlBuffer; // store command vector
    Eigen::MatrixXd stateBuffer; // store the states in buffer
    State currentState{}; // current state of the robot
    State predictedState{};

    StateTrajectory stateTrajectory{};
    StateGainMatrix StateGainsK{};

    Eigen::Matrix<double, 7, 7> PDGain;

    ControlTrajectory StateGainsk{};

    std::mutex mu;

    bool terminate{};

    RobotPublisherMPC() = default;
    RobotPublisherMPC(std::shared_ptr<Plant> robotPlant, int N_mpc, int T_N, double dt_)
    : m_robotPlant(robotPlant), dt(dt_), N_commands(N_mpc), N_Trajectory(T_N) {

        stateBuffer.resize(S, N_Trajectory + 1);
        stateBuffer.setZero();
        controlBuffer.resize(C, N_commands);
        controlBuffer.setZero();
        StateGainsK.resize(N_commands + 1);

        PDGain = 10 * Eigen::MatrixXd::Identity(7,7);
        StateGainsk.resize(C, N_commands);
        StateGainsk.setZero();

        for (auto& it : StateGainsK) {it.setZero();}
        stateTrajectory.resize(StateSize, N_commands + 1); stateTrajectory.setZero();
    }
    virtual ~RobotPublisherMPC() = default;

    RobotPublisherMPC(const RobotPublisherMPC &other) {}
    RobotPublisherMPC(RobotPublisherMPC &&other) = default;

    RobotPublisherMPC& operator=(const RobotPublisherMPC &other) {}
    RobotPublisherMPC& operator=(RobotPublisherMPC &&other) = default;


    virtual bool publishCommand(int i)
    {
      if (!isTerminate())
      {
        std::lock_guard<std::mutex> locker(mu);
        Control error = -1 * StateGainsK[i] * (stateTrajectory.col(i+1) - getCurrentState());
        Control pd    = 1 * PDGain * (stateTrajectory.col(i+1).head(7) - getCurrentState().head(7));

        u = controlBuffer.col(i) +  error;
        m_robotPlant->applyControl(u, stateTrajectory.col(i));

        // save the state
        saveState();
        command_step++;
      }

      return isTerminate();
    }

    // predicts the future state by simulating the dynamics
    const State& predictState(const State& currState, const ControlTrajectory& controlSequence, int time_steps_ahead) 
    {
      predictedState = currState;

      for (int i=0;i < time_steps_ahead;i++)
      {
        State f1 = m_robotPlant->m_plantDynamics->f(currState, controlSequence.col(i));
        State f2 = m_robotPlant->m_plantDynamics->f(currState + 0.5 * dt * f1, controlSequence.col(i));
        State f3 = m_robotPlant->m_plantDynamics->f(currState + 0.5 * dt * f2, controlSequence.col(i));
        State f4 = m_robotPlant->m_plantDynamics->f(currState + dt * f3, controlSequence.col(i));

        predictedState = predictedState + (dt/6) * (f1 + 2 * f2 + 2 * f3 + f4);
      }

      return predictedState;
    }


    virtual inline bool saveState() 
    {
      stateBuffer.col(command_step) = getCurrentState();
    }


    bool isTerminate() 
    {
      std::lock_guard<std::mutex> locker(mu);
      terminate = (command_step >  N_Trajectory) ? true : false;
      return terminate;
    }


    // set the control buffer from MPC optimizer
    bool setControlBuffer(const Eigen::MatrixXd &u)
    {
        std::lock_guard<std::mutex> locker(mu);
        controlBuffer = u;
        return true;
    }


    bool setOptimizerStatesGains(const Eigen::Ref<const StateTrajectory> &X, StateGainMatrix&& K, const commandVecTab_t& k)
    {
        StateGainsK = K;
        StateGainsk = k;
        stateTrajectory = X;
    }


    int getCurrentStep()
    {
        std::lock_guard<std::mutex> locker(mu);
        return command_step;
    }

    void setInitialState(const Eigen::Ref<const State> xinit) {
        std::lock_guard<std::mutex> locker(mu);
        m_robotPlant->setInitialState(xinit);
        stateBuffer.col(0) = xinit;
        ++command_step;
    }

    virtual const State& getCurrentState() 
    {
        return m_robotPlant->getCurrentState();
    }

    int getHorizonTimeSteps() const
    {
      return N_commands;
    }

    Eigen::MatrixXd& getStateTrajectory()
    {
      return stateBuffer;
    }

};
  
#endif // KUKAARM_H
