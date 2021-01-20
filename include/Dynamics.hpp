#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <cstdio>
#include <iostream>
#include <unsupported/Eigen/NumericalDiff>
#include <Eigen/Geometry>

#include <math.h>
#include <memory>
#include <functional>
#include <thread>

#include <mutex>

template<typename System, int StateDim, int ControlDim>
class Dynamics
{
    // enum{}

    template <class T, int S, int C>
    struct Differentiable
    {
        /*****************************************************************************/
        /*** Replicate Eigen's generic functor implementation to avoid inheritance ***/
        /*** We only use the fixed-size functionality ********************************/
        /*****************************************************************************/
        enum { InputsAtCompileTime = S + C, ValuesAtCompileTime = S };
        using Scalar        = T;
        using InputType     = Eigen::Matrix<T, InputsAtCompileTime, 1>;
        using ValueType     = Eigen::Matrix<T, ValuesAtCompileTime, 1>;
        using JacobianType  = Eigen::Matrix<T, ValuesAtCompileTime, InputsAtCompileTime>;
        int inputs() const { return InputsAtCompileTime; }
        int values() const { return ValuesAtCompileTime; }
        int operator()(const Eigen::Ref<const InputType> &xu, Eigen::Ref<ValueType> dx) const
        {
            dx =  dynamics_(xu.template head<S>(), xu.template tail<C>());
            return 0;
        }
        /*****************************************************************************/

        using DiffFunc = std::function<Eigen::Matrix<T, S, 1>(const Eigen::Matrix<T, S, 1>&, const Eigen::Matrix<T, C, 1>&)>;
        Differentiable(const DiffFunc &dynamics) : dynamics_(dynamics) {}
        Differentiable() = default;

    private:
        DiffFunc dynamics_;
    };



protected:
    using Scalar   = double;
    using Jacobian = Eigen::Matrix<double, StateDim, StateDim + ControlDim>;
    using State    = Eigen::Matrix<Scalar, StateDim, 1>;
    using Control  = Eigen::Matrix<Scalar, ControlDim, 1>;

    using JacobianState   = stateMatTab_t;
    using JacobianControl = stateR_commandC_tab_t;

    Scalar dt;
    int N;

    JacobianState fxList;
    JacobianControl fuList;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::shared_ptr<System> m_system;
    
    Jacobian jacobian;
    Differentiable<double, stateSize, commandSize> diff_;
    Eigen::NumericalDiff<Differentiable<double, stateSize, commandSize>, Eigen::Forward> num_diff_;

    Dynamics() = default;
    Dynamics(double timeStep, unsigned int Nsteps, const std::shared_ptr<System>& system) 
                    : m_system(system), 
                      diff_([this](const stateVec_t& x, const commandVec_t& u) -> stateVec_t{ return this->f(x, u); }), 
                      num_diff_(diff_), dt(timeStep), N(Nsteps) {}

    ~Dynamics() = default;
    Dynamics(const Dynamics &other) = default;

    Dynamics& operator=(const Dynamics &other) = default;

    virtual const State& f(const stateVec_t& x, const commandVec_t& tau) = 0;
    
    /* by default call the numerical differentiation*/
    virtual void fx(const stateVecTab_t& xList, const commandVecTab_t& uList)
    {
        // TODO parallalize here
        for (unsigned int k=0; k < N; k++) 
        {
            /* Numdiff Eigen */
            num_diff_.df((typename Differentiable<double, stateSize, commandSize>::InputType() << xList.col(k), uList.col(k)).finished(), jacobian);
            fxList[k] = jacobian.leftCols(stateSize) * dt + Eigen::Matrix<double, stateSize, stateSize>::Identity();
            fuList[k] = jacobian.rightCols(commandSize) * dt;
        }

    }

    const std::shared_ptr<System>& getSystem() {return m_system;}

};

#endif


