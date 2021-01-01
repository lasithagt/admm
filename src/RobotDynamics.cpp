#include "RobotDynamics.hpp"


RobotDynamics::RobotDynamics(double dt_, unsigned int N_, const std::shared_ptr<RobotAbstract>& kukaRobot, const ContactModel::SoftContactModel<double>& contact_model) 
: diff_([this](const stateVec_t& x, const commandVec_t& u) -> stateVec_t{ return this->f(x, u); }),
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

    // initialize contact model and the manipulator model
    m_contact_model = contact_model;
    m_kukaRobot     = kukaRobot;      
   
}

const RobotDynamics::State& RobotDynamics::f(const stateVec_t& X, const commandVec_t& tau)
{

    q  = X.head(NDOF);
    qd = X.segment(NDOF, NDOF);
    force_current = X.tail(3);

    // compute manipualator dynamics
    m_kukaRobot->getSpatialJacobian(q.data(), manip_jacobian);
    tau_ext = tau - 0*manip_jacobian.transpose().block(0, 0, NDOF, 3) * force_current;


    // std::cout << "dynamics" << std::endl;
    // std::cout << q.transpose() << std::endl;
    // std::cout << qd.transpose() << std::endl;

    m_kukaRobot->getForwardDynamics(q.data(), qd.data(), tau_ext, qdd);

    // std::cout << qdd.transpose() << std::endl;

    /*  contact model dynamics */
    if (CONTACT_EN)
    {
        // compute forward kinematics, velocity and accleration
        m_kukaRobot->getForwardKinematics(q.data(), qd.data(), qdd.data(), poseM, poseP, vel, accel, true);

        // contact model dynamics
        // std:;cout << "vel" << std::endl;
        // std::cout << vel << std::endl;
        // std::cout << accel << std::endl;
        m_contact_model.df(H_c, poseP, poseM, vel, accel, force_current, force_dot);

    } else {   
        force_dot.setZero();
        force_current.setZero();
    }

    xdot_new << qd, qdd, force_dot;
    return xdot_new;
}


// TODO: numdiff here
// void RobotDynamics::fx(const stateVecTab_t& xList, const commandVecTab_t& uList)
// {
//     // TODO parallalize here
//     for (unsigned int k=0; k < N; k++) 
//     {
//         /* Numdiff Eigen */
//         num_diff_.df((typename Differentiable<double, stateSize, commandSize>::InputType() << xList.col(k), uList.col(k)).finished(), j_);
//         fxList[k] = j_.leftCols(stateSize) * dt + Eigen::Matrix<double, stateSize, stateSize>::Identity();
//         fuList[k] = j_.rightCols(commandSize) * dt;
//     }

// }

// TODO: autodiff here
void RobotDynamics::fx(const stateVecTab_t& xList, const commandVecTab_t& uList)
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


// TODO: optimize the code here, move semantics
commandVec_t RobotDynamics::getLowerCommandBounds()
{
    return lowerCommandBounds;
}

commandVec_t RobotDynamics::getUpperCommandBounds()
{
    return upperCommandBounds;
}

const stateMatTab_t& RobotDynamics::getfxList()
{
    return fxList;
}

const stateR_commandC_tab_t& RobotDynamics::getfuList()
{
    return fuList;
}

