
#ifndef ROBOTDYNAMICS_H
#define ROBOTDYNAMICS_H

#include "config.h"
#include "soft_contact_model.hpp"
#include "RobotAbstract.h"

#include <chrono>

#include <ct/rbd/rbd.h>
#include <ct/optcon/optcon.h>

#include "KUKA.h"
#include "KUKASoftContactFDSystem.h"
#include "RobCodGen/codegen/KUKASoftContactSystemLinearizedForward.h"

#include "dynamics.hpp"

#include <mutex>


// template<typename System, int StateDim, int ControlDim>
class RobotDynamics : public admm::Dynamics<RobotAbstract, stateSize, commandSize>
{

    using Jacobian        = Eigen::Matrix<double, stateSize, stateSize + commandSize>;
    using State           = stateVec_t;
    using Control         = commandVec_t;
    using JacobianState   = typename admm::Dynamics<RobotAbstract, stateSize, commandSize>::JacobianState;
    using JacobianControl = typename admm::Dynamics<RobotAbstract, stateSize, commandSize>::JacobianControl;

    typedef ct::rbd::KUKASoftContactFDSystem<ct::rbd::KUKA::Dynamics> KUKASystem;
    const size_t STATE_DIM = KUKASystem::STATE_DIM;
    const size_t CONTROL_DIM = KUKASystem::CONTROL_DIM;

private:

    Control lowerCommandBounds;
    Control upperCommandBounds;

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


    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    std::chrono::duration<float, std::nano> elapsed;

public:
    RobotDynamics() 
    {
        std::cout << "Initilized the Robot Dynamic Model..." << std::endl;
    }
    RobotDynamics(double timeStep, unsigned int Nsteps, const std::shared_ptr<RobotAbstract>& kukaRobot, const ContactModel::SoftContactModel<double>& contact_model) 
                    : admm::Dynamics<RobotAbstract, stateSize, commandSize>(timeStep, Nsteps, kukaRobot), m_kukaRobot(kukaRobot), m_contact_model(contact_model)
                      
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
    RobotDynamics(const RobotDynamics &other) {};
    RobotDynamics& operator=(const RobotDynamics &other) {};

    const State& f(const stateVec_t& x, const commandVec_t& tau) override
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

        // contact model dynamics
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

    void fx(const stateVecTab_t& xList, const commandVecTab_t& uList) override
    {
        // TODO parallalize here
        for (unsigned int k=0; k < N; k++) 
        {
            x = xList.col(k); u = uList.col(k);
            x(16) += 0.000000001;

            auto A_gen = kukaLinear.getDerivativeState(x, u, 0.0);
            auto B_gen = kukaLinear.getDerivativeControl(x, u, 0.0);

            this->fxList[k] = A_gen * this->dt + Eigen::Matrix<double, stateSize, stateSize>::Identity();
            this->fuList[k] = B_gen * this->dt;
        }
    }

    const Control& getLowerCommandBounds() const {return lowerCommandBounds;}
    const Control& getUpperCommandBounds() const {return upperCommandBounds;}
    const JacobianState& getfxList() const override {return this->fxList;}
    const JacobianControl& getfuList() const override {return this->fuList;}
};



#endif // KUKAARM_H
