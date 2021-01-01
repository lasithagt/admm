    
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


#ifndef DEBUG_KUKA_ARM
#define DEBUG_KUKA_ARM 1
#else
    #if PREFIX1(DEBUG_KUKA_ARM)==1
    #define DEBUG_KUKA_ARM 1
    #endif
#endif

#define TRACE_KUKA_ARM(x) do { if (DEBUG_KUKA_ARM) printf(x);} while (0)

using namespace std;

class RobotDynamics
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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


protected:
    commandVec_t lowerCommandBounds;
    commandVec_t upperCommandBounds;

    stateMat_t fx_;
    stateR_commandC_t fu_;

    stateMatTab_t fxList;
    stateR_commandC_tab_t fuList;

private:
    double dt;
    int N;
    bool initial_phase_flag_;

public:
    std::shared_ptr<RobotAbstract> m_kukaRobot;
    ct::models::KUKA::KUKASoftContactSystemLinearizedForward kukaLinear;
    ct::core::StateVector<KUKASystem::STATE_DIM> x;
    ct::core::ControlVector<KUKASystem::CONTROL_DIM> u; 


    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    std::chrono::duration<float, std::nano> elapsed;

private:
    
    ContactModel::SoftContactModel<double> m_contact_model;
    stateR_half_commandC_t Bu; //input mapping
    stateVec_t xdot_new;
    stateVec_half_t vd;

    Eigen::VectorXd q, qd, qdd, tau_ext;
    Eigen::Vector3d force_current, accel, vel, poseP;
    Eigen::Matrix<double,3,3> poseM;
    Eigen::Matrix3d H_c;
    Eigen::MatrixXd manip_jacobian;

    Eigen::Vector3d force_dot;

    stateVecTab_t xList_;
    commandVecTab_t uList_;


    bool debugging_print;

    Jacobian j_;
    Differentiable<double, stateSize, commandSize> diff_;
    Eigen::NumericalDiff<Differentiable<double, stateSize, commandSize>, Eigen::Forward> num_diff_;

public:
    RobotDynamics(){}
    RobotDynamics(double dt_, unsigned int N_, const std::shared_ptr<RobotAbstract>& kukaRobot, const ContactModel::SoftContactModel<double>& contact_model) 
: diff_([this](const stateVec_t& x, const commandVec_t& u) -> stateVec_t{ return this->f(x, u); }) {
        q.resize(NDOF), qd.resize(NDOF);
        qdd.resize(NDOF);
        fxList.resize(N + 1), fuList.resize(N);
        tau_ext.resize(NDOF);
        manip_jacobian.resize(6, NDOF);
        poseM.setZero();
        H_c << 1, 0, 0, 0, 1, 0, 0, 0, 1;
        xdot_new.setZero();

        // initialize contact model and the manipulator model
        m_contact_model = contact_model;
        m_kukaRobot     = kukaRobot;      
   
    }

    ~RobotDynamics(){}
    // RobotDynamics(double dt, unsigned int N,  const std::shared_ptr<RobotAbstract>& robot, const ContactModel::SoftContactModel<double>& contact_model);
    const State& f(const stateVec_t& X, const commandVec_t& tau){
        q  = X.head(NDOF);
        qd = X.segment(NDOF, NDOF);
        force_current = X.tail(3);

        // compute manipualator dynamics
        m_kukaRobot->getSpatialJacobian(q.data(), manip_jacobian);
        tau_ext = tau - 0*manip_jacobian.transpose().block(0, 0, NDOF, 3) * force_current;

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

    commandVec_t getLowerCommandBounds() {return lowerCommandBounds;}
    commandVec_t getUpperCommandBounds() {return upperCommandBounds;}
    const stateMatTab_t& getfxList() {return fxList;}
    const stateR_commandC_tab_t& getfuList(){return fuList;}
};



#endif // KUKAARM_H
