
#ifndef ROBOTDYNAMICS_H
#define ROBOTDYNAMICS_H

#include "config.h"
#include "SoftContactModel.h"
#include "KukaModel.h"

#include <cstdio>
#include <iostream>
#include <unsupported/Eigen/NumericalDiff>
#include <Eigen/Geometry>
#include <chrono>

#include <math.h>
#include <memory>
#include <functional>
#include <thread>

#include <ct/rbd/rbd.h>
#include <ct/optcon/optcon.h>

#include "KUKA.h"

#include "KUKASoftContactFDSystem.h"
#include "RobCodGen/codegen/KUKASoftContactSystemLinearizedForward.h"

#include <mutex>


class RobotDynamics
{
    

    using Jacobian = Eigen::Matrix<double, stateSize, stateSize + commandSize>;
    using State    = stateVec_t;
    using Control  = commandVec_t;

    typedef ct::rbd::KUKASoftContactFDSystem<ct::rbd::KUKA::Dynamics> KUKASystem;
    const size_t STATE_DIM = KUKASystem::STATE_DIM;
    const size_t CONTROL_DIM = KUKASystem::CONTROL_DIM;

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

    using Scalar          =  double;
    using JacobianState   = stateMatTab_t;
    using JacobianControl = stateR_commandC_tab_t;
    // typedef ContactModel::SoftContactModel<Scalar> ContactModel;

private:
    Scalar dt;
    int N;

    Control lowerCommandBounds;
    Control upperCommandBounds;

    JacobianState fxList;
    JacobianControl fuList;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::mutex mu;
    std::shared_ptr<RobotAbstract> m_kukaRobot;
    ContactModel::SoftContactModel<Scalar> m_contact_model;
    ct::models::KUKA::KUKASoftContactSystemLinearizedForward kukaLinear;
    ct::core::StateVector<KUKASystem::STATE_DIM> x;
    ct::core::ControlVector<KUKASystem::CONTROL_DIM> u; 
    
    stateVec_t xdot_new;

    Eigen::VectorXd q, qd, qdd, tau_ext;
    Eigen::Vector3d force_current, accel, vel, poseP;
    Eigen::Matrix<double,3,3> poseM;
    Eigen::Matrix3d H_c;
    Eigen::MatrixXd manip_jacobian;
    Control Kv;
    Control dynamic_friction;
    Eigen::Vector3d force_dot;

    Jacobian jacobian;
    Differentiable<double, stateSize, commandSize> diff_;
    Eigen::NumericalDiff<Differentiable<double, stateSize, commandSize>, Eigen::Forward> num_diff_;


    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    std::chrono::duration<float, std::nano> elapsed;

public:
    RobotDynamics() 
    {
        std::cout << "Initilized the Robot Dynamic Model..." << std::endl;
    }
    RobotDynamics(double dt_, unsigned int N_, const std::shared_ptr<RobotAbstract>& kukaRobot, const ContactModel::SoftContactModel<double>& contact_model) 
                    : m_kukaRobot(kukaRobot), 
                      m_contact_model(contact_model), 
                      diff_([this](const stateVec_t& x, const commandVec_t& u) -> stateVec_t{ return this->f(x, u); }), 
                      num_diff_(diff_), dt(dt_), N(N_) 
    {
        q.resize(NDOF), qd.resize(NDOF);
        qdd.resize(NDOF);
        fxList.resize(N + 1), fuList.resize(N);
        tau_ext.resize(NDOF);
        manip_jacobian.resize(6, NDOF);
        poseM.setZero();
        H_c << 1, 0, 0, 0, 1, 0, 0, 0, 1;

        xdot_new.setZero();
        Kv << 0.5, 0.5, 0.5, 0.3, 1, 0.5, 0.2;
        RobotDynamics();
   
    }

    ~RobotDynamics() = default;
    RobotDynamics(const RobotDynamics &other)  
    {
        // *this = other;
    }

    RobotDynamics& operator=(const RobotDynamics &other) 
    {
        // *this = other;
        // return *this;
    }


    // RobotDynamics(double dt, unsigned int N,  const std::shared_ptr<RobotAbstract>& robot, const ContactModel::SoftContactModel<double>& contact_model);
    const State& f(const stateVec_t& x, const commandVec_t& tau)
    {
        std::lock_guard<std::mutex> lk(mu);
        q  = x.head(NDOF);
        qd = x.segment(NDOF, NDOF);
        force_current = x.tail(3);

        // compute manipualator dynamics
        m_kukaRobot->getSpatialJacobian(q.data(), manip_jacobian);

        dynamic_friction = (-1) * Kv.asDiagonal() * qd;

        tau_ext = tau + dynamic_friction - 0 * manip_jacobian.transpose().block(0, 0, NDOF, 3) * force_current;
        m_kukaRobot->getForwardDynamics(q.data(), qd.data(), tau_ext, qdd);

        /*  contact model dynamics */
        if (CONTACT_EN)
        {
            // compute forward kinematics, velocity and accleration
            m_kukaRobot->getForwardKinematics(q.data(), qd.data(), qdd.data(), poseM, poseP, vel, accel, true);

            // contact model dynamics
            m_contact_model.df(H_c, poseP, poseM, vel, accel, force_current, force_dot);

        } else {   
            force_dot.setZero();
            force_current.setZero();
        }

        xdot_new << qd, qdd, force_dot;
        return xdot_new;
    }

    void fx(const stateVecTab_t& xList, const commandVecTab_t& uList) 
    {
        // TODO parallalize here
        for (unsigned int k=0; k < N; k++) 
        {
            x = xList.col(k); u = uList.col(k);
            x(16) += 0.000000001;

            auto A_gen = kukaLinear.getDerivativeState(x, u, 0.0);
            auto B_gen = kukaLinear.getDerivativeControl(x, u, 0.0);

            fxList[k] = A_gen * dt + Eigen::Matrix<double, stateSize, stateSize>::Identity();
            fuList[k] = B_gen * dt;
        }
    }

    // void fx(const stateVecTab_t& xList, const commandVecTab_t& uList)
    // {
    //     // TODO parallalize here
    //     for (unsigned int k=0; k < N; k++) 
    //     {
    //         /* Numdiff Eigen */
    //         num_diff_.df((typename Differentiable<double, stateSize, commandSize>::InputType() << xList.col(k), uList.col(k)).finished(), jacobian);
    //         fxList[k] = jacobian.leftCols(stateSize) * dt + Eigen::Matrix<double, stateSize, stateSize>::Identity();
    //         fuList[k] = jacobian.rightCols(commandSize) * dt;
    //     }

    // }
    const Control& getLowerCommandBounds() const {return lowerCommandBounds;}
    const Control& getUpperCommandBounds() const {return upperCommandBounds;}
    const JacobianState& getfxList() const {return fxList;}
    const JacobianControl& getfuList() const {return fuList;}
};



#endif // KUKAARM_H
