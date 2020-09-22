#include "kuka_arm.h"
#include <Eigen/Geometry>


KukaArm::KukaArm(double& dt_, unsigned int& N_, std::shared_ptr<KUKAModelKDL>& kukaRobot, ContactModel::SoftContactModel& contact_model) 
: diff_([this](const stateVec_t& x, const commandVec_t& u) -> stateVec_t{ return this->kuka_arm_dynamics(x, u); }),
      num_diff_(diff_), dt(dt_), N(N_)
{
    globalcnt = 0;

    q.resize(NDOF), qd.resize(NDOF);

    fxList.resize(N + 1), fuList.resize(N);

    tau_ext.resize(NDOF);
    manip_jacobian.resize(6, NDOF);

    // cartesian mass matrix
    H_c << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Bu.setZero();
    xdot_new.setZero();

    debugging_print = 0, finalTimeProfile.counter0_ = 0, finalTimeProfile.counter1_ = 0, finalTimeProfile.counter2_ = 0;
    finalTimeProfile.time_period1 = 0, finalTimeProfile.time_period2 = 0, finalTimeProfile.time_period3 = 0, finalTimeProfile.time_period4 = 0;

    initial_phase_flag_ = 1;
    q.resize(NDOF), qd.resize(NDOF);

    // initialize contact model and the manipulator model
    contact_model0 = &contact_model;
    if (initial_phase_flag_ == 1)
    {
        kukaRobot_    = kukaRobot;      
        initial_phase_flag_ = 0;
    }
}

stateVec_t KukaArm::kuka_arm_dynamics(const stateVec_t& X, const commandVec_t& tau)
{
    // struct timeval tbegin_dynamics, tend_dynamics;
    // gettimeofday(&tbegin_dynamics,NULL);

    finalTimeProfile.counter0_ += 1;

    if(finalTimeProfile.counter0_ == 10)
        gettimeofday(&tbegin_period, NULL);


    q  = X.head(NDOF);
    qd = X.segment(NDOF, NDOF);
    force_current = X.tail(3);


    Eigen::Vector3d force_dot;
    Eigen::Vector3d poseM_vec;
    poseM_vec.setZero();

    // compute manipualator dynamics
    kukaRobot_->getSpatialJacobian(q.data(), manip_jacobian);
    tau_ext = tau; // + manip_jacobian.transpose().block(0, 0, NDOF, 3) * force_current;

    kukaRobot_->getForwardDynamics(q.data(), qd.data(), tau_ext, qdd);

    /*  contact model dynamics */
    if (CONTACT_EN)
    {
        // compute forward kinematics, velocity and accleration
        kukaRobot_->getForwardKinematics(q.data(), qd.data(), qdd.data(), poseM, poseP, vel, accel, true);

        // contact model dynamics
        contact_model0->df(H_c, poseP, poseM_vec, vel, accel, force_current, force_dot);
        // std::cout << "Force_dot\n" << force_dot << std::endl;
    } else 
    {   
        // for testing
        force_dot.setZero();
        force_current.setZero();
    }

    xdot_new << qd, qdd, force_dot;


    if (finalTimeProfile.counter0_ == 10)
    {
        gettimeofday(&tend_period,NULL);
        finalTimeProfile.time_period1 += (static_cast<double>(1000.0*(tend_period.tv_sec-tbegin_period.tv_sec)+((tend_period.tv_usec-tbegin_period.tv_usec)/1000.0)))/1000.0;
    }
    

    if (globalcnt < 40) {
        globalcnt += 1;
    }

    return xdot_new;
}


KukaArm::timeprofile KukaArm::getFinalTimeProfile()
{    
    return finalTimeProfile;
}

void KukaArm::compute_dynamics_jacobian(const stateVecTab_t& xList, const commandVecTab_t& uList)
{

    // // for a positive-definite quadratic, no control cost (indicated by the iLQG function using nans), is equivalent to u=0
    if(debugging_print) TRACE_KUKA_ARM("initialize dimensions\n");
    // unsigned int Nl = xList.cols();

    if(debugging_print) TRACE_KUKA_ARM("compute cost function\n");

    for (unsigned int k=0; k < N; k++) 
    {
        /* Numdiff Eigen */
        num_diff_.df((typename Differentiable<double, stateSize, commandSize>::InputType() << xList.col(k), uList.col(k)).finished(), j_);

        fxList[k] = j_.leftCols(stateSize) * dt + Eigen::Matrix<double, stateSize, stateSize>::Identity();
        fuList[k] = j_.rightCols(commandSize) * dt;
    }

    
    if(debugging_print) TRACE_KUKA_ARM("finish kuka_arm_dyn_cst\n");
}




commandVec_t& KukaArm::getLowerCommandBounds()
{
    return lowerCommandBounds;
}

commandVec_t& KukaArm::getUpperCommandBounds()
{
    return upperCommandBounds;
}

stateMatTab_t& KukaArm::getfxList()
{
    return fxList;
}

stateR_commandC_tab_t& KukaArm::getfuList()
{
    return fuList;
}

