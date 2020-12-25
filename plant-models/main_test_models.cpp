#include "KukaModel.h"
#include "models.h"

#include "RobCodGenModel.h"

#include <memory>
#include <ct/rbd/rbd.h>

#include "RobCodGen/KUKA.h"
#include "ct/rbd/robot/Dynamics.h"
#include "ct/rbd/robot/Kinematics.h"

using namespace ct::rbd;


// Test scripts
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");


int main() {

	RobCodGenModel robot_A = RobCodGenModel();
	robot_A.initRobot();

	double q[7] = {0,1,0,0,0,0,0}; 
	double qd[7] = {0,0,0,0,0,0,0}; 
	double qdd[7] = {0,0,0,0,0,0,0}; 
	Eigen::MatrixXd jac;
	jac.resize(6,7);
	robot_A.getSpatialJacobian(q, jac);

	std::cout << jac << std::endl;

	Eigen::Matrix<double,3,3> poseM;
	Eigen::Vector3d poseP;
	Eigen::Vector3d vel;
	Eigen::Vector3d accel;

	robot_A.getForwardKinematics(q, qd, qdd, poseM, poseP, vel, accel, true);

	std::cout << poseP << std::endl;

	Eigen::VectorXd torque_ext;
	torque_ext.resize(7);
	torque_ext.setZero();
	Eigen::VectorXd xd;
	xd.resize(7);

	robot_A.getForwardDynamics(q, qd, torque_ext, xd);

	std::cout << xd << std::endl;

	Eigen::VectorXd torque_u;
	torque_u.resize(7);
	robot_A.getInverseDynamics(q, qd, qdd, torque_u);

	std::cout << torque_u << std::endl;


	robot_A.getGravityVector(q, torque_u);

	std::cout << torque_u << std::endl;

	// std::shared_ptr<KUKA::Kinematics> kyn(new KUKA::Kinematics);

 //    KUKA::Dynamics testdynamics = KUKA::Dynamics(kyn);

 //    using control_vector_t = typename KUKA::Dynamics::control_vector_t;
 //    using ForceVector_t = typename KUKA::Dynamics::ForceVector_t;
 //    using JointState_t = typename KUKA::Dynamics::JointState_t;
 //    using JointAcceleration_t = typename KUKA::Dynamics::JointAcceleration_t;
 //    using ExtLinkForces_t = typename KUKA::Dynamics::ExtLinkForces_t;

 //    JointState<KUKA::Kinematics::NJOINTS> irb_state;
 //    irb_state.setZero();

 //    control_vector_t torque_u = control_vector_t::Zero();
 //    ExtLinkForces_t ext_forces;
 //    ext_forces = ForceVector_t::Zero();
 //    JointAcceleration_t irb_xd;
 //    testdynamics.FixBaseForwardDynamics(irb_state, torque_u, ext_forces, irb_xd);
 //    JointAcceleration_t qdd;

 //    // id
 //    testdynamics.FixBaseID(irb_state, qdd, ext_forces, torque_u);

 //    // Kinematics, position EE
 //    KUKA::Kinematics kyn_;
 //    EndEffector<KUKA::Kinematics::NJOINTS> eeTest;
 //    for (size_t i = 0; i < KUKA::Kinematics::NUM_EE; i++) {
 //        eeTest = kyn_.getEndEffector(i);
 //    }
 //    JointState<KUKA::Kinematics::NJOINTS> kukaJointState;
 //    RigidBodyPose kukaPose;
 //    kukaJointState.setZero();
 //    kukaPose.setIdentity();
 //    size_t ind = 0;
 //    kindr::Position3D pos = kyn_.getEEPositionInWorld(ind, kukaPose, kukaJointState.getPositions());
 //    std::cout << pos << std::endl;


 //    // Kinematics, velocity EE
 //    RigidBodyState baseState;
 //    baseState.setIdentity();
 //    Eigen::Matrix<double,2*KUKA::Kinematics::NJOINTS, 1>& state_ = kukaJointState.toImplementation();
 //    double qd[7] = {0,0,0,2,0,0,0}; 
 //    memcpy(state_.tail(7).data(), qd, 7*sizeof(double));
 //    // state_(10,0) = 2;
 //    // std::cout << kukaJointState.getVelocity() << std::endl;
 //    RBDState<KUKA::Kinematics::NJOINTS> RBD = RBDState<KUKA::Kinematics::NJOINTS>(baseState, kukaJointState);
 //    kindr::Velocity3D vel = kyn_.getEEVelocityInBase(ind, RBD);
 //    std::cout << vel << std::endl;


 //    // Jacobian
 //    KUKA::Kinematics::Jacobian jac;
 //    size_t ee_id = 0;
 //    jac = kyn_.getJacobianBaseEEbyId(ee_id, RBD);

 //    std::cout << jac << std::endl;


}


