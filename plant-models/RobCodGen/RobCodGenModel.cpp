#include "RobCodGenModel.h"
#include <mutex>


// std::mutex mu;

RobCodGenModel::RobCodGenModel() : robot_state(joint_state.toImplementation())
{
    // initRobot();
}

RobCodGenModel::~RobCodGenModel() {};

int RobCodGenModel::initRobot() 
{
    base_pose.setIdentity();
    base_state.setIdentity();
    RBD = RBDState<KUKA::Kinematics::NJOINTS>(base_state, joint_state);
    robot_ee = m_kyn.getEndEffector(0);

    kyn_ptr = std::make_shared<KUKA::Kinematics>(m_kyn);
    m_dyn = KUKA::Dynamics(kyn_ptr);

    id_qdd.setZero();

    gravity_u.resize(KUKA::Kinematics::NJOINTS);


    return 0;
    
}

void RobCodGenModel::getForwardKinematics(double* q, double* qd, double *qdd, Eigen::Matrix<double,3,3>& poseM, Eigen::Vector3d& poseP, Eigen::Vector3d& vel, Eigen::Vector3d& accel, bool computeOther)
{
    // set the states
    memcpy(robot_state.head(7).data(), q, 7*sizeof(double));
    memcpy(robot_state.tail(7).data(), qd, 7*sizeof(double));

    // Kinematics, position
    size_t ind = 0;
    ee_pose = m_kyn.getEEPoseInWorld(ind, base_pose, joint_state.getPositions());
    poseM = ee_pose.getRotationMatrix().matrix();
    poseP = ee_pose.position().vector();

    Jc_t.ee_indices_.push_back(0);
    Jc_t.eeInContact_[0] = true;

    RBD.joints() = joint_state;

    // Kinematics, velocity
    vel = m_kyn.getEEVelocityInBase(ind, RBD).vector();

    // Kinematics, acceleration
    if (computeOther == true) {
        Eigen::MatrixXd temp(6,7);
        Eigen::VectorXd temp_qdd(7);
        memcpy(temp_qdd.data(), qdd, 7 * sizeof(double));

        getSpatialJacobian(q, temp);
        accel = temp.block(0,0,3,7) * temp_qdd;

        getSpatialJacobianDot(q, qd, temp);
        accel += temp.block(0,0,3,7) * joint_state.getVelocities();
    }
    // TODO

}

/* given q, qdot, qddot, outputs torque output*/
void RobCodGenModel::getInverseDynamics(double* q, double* qd, double* qdd, Eigen::VectorXd& torque)
{
    // set the states
    memcpy(robot_state.head(7).data(), q, 7*sizeof(double));
    memcpy(robot_state.tail(7).data(), qd, 7*sizeof(double));

    // testdynamics.FixBaseID(irb_state, qdd, ext_forces, torque_u);
    m_dyn.FixBaseID(robot_state, id_qdd, torque_u);
    memcpy(torque.data(), torque_u.data(), 7*sizeof(double));

}

void RobCodGenModel::getForwardDynamics(double* q, double* qd, const Eigen::VectorXd& torque_ext, Eigen::VectorXd& xd)
{
    // set the states
    memcpy(robot_state.head(7).data(), q, 7*sizeof(double));
    memcpy(robot_state.tail(7).data(), qd, 7*sizeof(double));

    // do gravity compensation
    getGravityVector(q, gravity_u);
    gravity_u = torque_ext + gravity_u.eval();
    memcpy(torque_u.data(), gravity_u.data(), 7*sizeof(double));
    
    m_dyn.FixBaseForwardDynamics(joint_state, torque_u, qdd);
    memcpy(xd.data(), qdd.getAcceleration().data(), 7*sizeof(double));

}

void RobCodGenModel::getMassMatrix(double* q, Eigen::MatrixXd& massMatrix)
{

} 

void RobCodGenModel::getCoriolisMatrix(double* q, double* qd, Eigen::VectorXd& coriolis) // change
{
    double qdd[7] = {0, 0, 0, 0, 0, 0, 0};

}
 

void RobCodGenModel::getGravityVector(double* q, Eigen::VectorXd& gravityTorque)
{
    double qd[7] = {0, 0, 0, 0, 0, 0, 0};
    getInverseDynamics(q, qd, qd, gravityTorque);
}
 

void RobCodGenModel::getSpatialJacobian(double* q, Eigen::MatrixXd& jacobian)
{
    memcpy(robot_state.head(7).data(), q, 7*sizeof(double));
    size_t ee_id = 0;
    RBD.joints() = joint_state;

    jac = m_kyn.getJacobianBaseEEbyId(ee_id, RBD);

    jacobian.block(0,0,3,7) = jac.template bottomRows<3>();
    jacobian.block(3,0,3,7) = jac.template topRows<3>();

    // chnage this
    // memcpy(jacobian.data(), jac.data(), 42*sizeof(double));

} 

void RobCodGenModel::getSpatialJacobianDot(double* q, double* qd, Eigen::MatrixXd& jacobianDot)
{   
    memcpy(robot_state.head(7).data(), q, 7*sizeof(double));
    memcpy(robot_state.tail(7).data(), qd, 7*sizeof(double));
    RBD.joints() = joint_state;


    Eigen::Matrix<double, 3, KUKA::Kinematics::NJOINTS> Jc_Rotational, Jc_Translational, dJdt;
    Eigen::Matrix<double, 3, 1> dJidqj;

    dJdt.setZero();
    size_t ee_id = 0;

    jac = m_kyn.getJacobianBaseEEbyId(ee_id, RBD);

    // Collect current contact Jacobians
    // Jc_geometric  = m_dyn.robcogen().getJacobianBaseEEbyId(ee_indices_[0], state.joints().getPositions());
    Jc_Rotational = jac.template topRows<3>();
    Jc_Translational = jac.template bottomRows<3>();

    // Compute dJdt for the joint columns
    for (size_t i = 0; i < KUKA::Kinematics::NJOINTS; i++)
    {  // Loop over columns i of dJdt_joints
     for (size_t j = 0; j < KUKA::Kinematics::NJOINTS; j++)
     {  // Loop over derivative w.r.t each joint j and sum
         if (i >= j)
         {
             dJidqj = (Jc_Rotational.template block<3, 1>(0, j))
                          .template cross(Jc_Translational.template block<3, 1>(0, i));
         }
         else
         {  // i < j
             dJidqj = (Jc_Rotational.template block<3, 1>(0, i))
                          .template cross(Jc_Translational.template block<3, 1>(0, j));
         }
         dJdt.template block<3, 1>(0, i) += dJidqj * joint_state.getVelocities()(j);
     }
    }

    // Fill in the rows of the full dJdt
    jacobianDot.block(0,0,3,7) = dJdt;
}

void RobCodGenModel::ik()
{
    
}
