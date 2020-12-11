    
#ifndef KUKAARM_H
#define KUKAARM_H

#include "config.h"
#include "SoftContactModel.h"

#include "KukaModel.h"


#include <cstdio>
#include <iostream>
#include <Eigen/Dense>
#include <unsupported/Eigen/NumericalDiff>

#include <math.h>

#include <memory>
#include <functional>
// #include <mutex>
// std::mutex mtx;

#define pi 3.141592653

#ifndef DEBUG_KUKA_ARM
#define DEBUG_KUKA_ARM 1
#else
    #if PREFIX1(DEBUG_KUKA_ARM)==1
    #define DEBUG_KUKA_ARM 1
    #endif
#endif

#define TRACE_KUKA_ARM(x) do { if (DEBUG_KUKA_ARM) printf(x);} while (0)

using namespace Eigen;
using namespace std;


class KukaArm
{
    using Jacobian = Eigen::Matrix<double, stateSize, stateSize + commandSize>;

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

    /* --------------- to evaluate time profiles ------------------------*/
    struct timeprofile
    {
        double time_period1, time_period2, time_period3, time_period4;
        unsigned int counter0_, counter1_, counter2_;
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
    struct timeprofile finalTimeProfile;
    struct timeval tbegin_period, tend_period, tbegin_period4, tend_period4; //tbegin_period2, tend_period2, tbegin_period3, tend_period3, 

public:
    static const double mc, mp, l, g;
    unsigned int globalcnt;
    std::shared_ptr<RobotAbstract> kukaRobot_;

private:
    
    ContactModel::SoftContactModel m_contact_model;
    stateR_half_commandC_t Bu; //input mapping
    stateVec_t xdot_new;
    stateVec_half_t vd;
    stateVecTab_half_t vd_thread;
    stateVecTab_t Xdot_new_thread;

    Eigen::VectorXd q, qd, qdd, tau_ext;
    Eigen::Vector3d force_current, accel, vel, poseP;
    Eigen::Matrix<double,3,3> poseM;
    Eigen::Matrix3d H_c;
    Eigen::MatrixXd manip_jacobian;

    std::vector<Eigen::VectorXd> q_thread, qd_thread;
    bool debugging_print;

    Jacobian j_;
    Differentiable<double, stateSize, commandSize> diff_;
    Eigen::NumericalDiff<Differentiable<double, stateSize, commandSize>, Eigen::Forward> num_diff_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KukaArm() {}

    ~KukaArm(){};
    KukaArm(double dt, unsigned int N,  std::shared_ptr<RobotAbstract>& kukaRobot, const ContactModel::SoftContactModel& contact_model);
    stateVec_t kuka_arm_dynamics(const stateVec_t& X, const commandVec_t& tau);
    void compute_dynamics_jacobian(const stateVecTab_t& xList, const commandVecTab_t& uList);
    struct timeprofile getFinalTimeProfile();

    commandVec_t& getLowerCommandBounds();
    commandVec_t& getUpperCommandBounds();
    stateMatTab_t& getfxList();
    stateR_commandC_tab_t& getfuList();
};



#endif // KUKAARM_H
