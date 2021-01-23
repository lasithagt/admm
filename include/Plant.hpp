#ifndef PLANT_H
#define PLANT_H

#include <Eigen/Dense>
#include <memory>


template<int S, int C>
class Plant
{
public:
    enum { StateSize = S, ControlSize = C };
    using Scalar   = double;
    using State    = Eigen::Matrix<Scalar, StateSize, 1>;
    using Control  = Eigen::Matrix<Scalar, ControlSize, 1>;

    // std::shared_ptr<Dynamics> m_plantDynamics;
    Scalar dt;
    State currentState;


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Plant() = default;
    Plant(Scalar TimeStep)
    : dt(TimeStep) {currentState.setZero();}

    Plant(const Plant &other) = default;
    Plant(Plant &&other) = default;

    Plant& operator=(const Plant &other) = default;
    Plant& operator=(Plant &&other) = default;
    ~Plant() = default;


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
    virtual bool applyControl(const Eigen::Ref<const Control> &u, const Eigen::Ref<const State> &x) {};


    bool setInitialState(const Eigen::Ref<const State> &x)
    {
        currentState = x;
        return true;
    }

    virtual const State& getCurrentState()
    {
        return currentState;
    }
};

#endif // KUKAARM_H

