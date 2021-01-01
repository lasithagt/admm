#include "KukaModel.h"
#include "models.h"

#include "RobCodGenModel.h"

#include <memory>
#include <ct/rbd/rbd.h>

#include "RobCodGen/KUKA.h"
#include "ct/rbd/robot/Dynamics.h"
#include "ct/rbd/robot/Kinematics.h"

#include <ct/optcon/optcon.h>

#include <ct/rbd/robot/jacobian/OperationalJacobianBase.h>
#include <ct/rbd/operationalSpace/rigid_body/OperationalModel.h>
#include <ct/rbd/operationalSpace/rigid_body/OperationalModelRBD.h>

#include "KUKASoftContactFDSystem.h"
#include "RobCodGen/codegen/KUKASoftContactSystemLinearizedForward.h"

using namespace ct::rbd;


// Test scripts
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");


void TEST_RCG () {
	RobCodGenModel robot_A = RobCodGenModel();
	robot_A.initRobot();

	double q[7] = {0,0,0,0,0,0,0}; 
	double qd[7] = {0,1,0,0,0,0,0}; 
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

	std::cout << "Velocity" << std::endl;
	std::cout << vel << std::endl;

	std::cout << "Acceleration" << std::endl;
	std::cout << accel << std::endl;

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


	Eigen::MatrixXd jacobianDot(6,7);
	robot_A.getSpatialJacobianDot(q, qd, jacobianDot);


	std::cout << jacobianDot << std::endl;



}

void TEST_DYNAMICS() {
	std::shared_ptr<KUKA::Kinematics> kyn(new KUKA::Kinematics);

   KUKA::Dynamics testdynamics = KUKA::Dynamics(kyn);

   using control_vector_t = typename KUKA::Dynamics::control_vector_t;
   using ForceVector_t = typename KUKA::Dynamics::ForceVector_t;
   using JointState_t = typename KUKA::Dynamics::JointState_t;
   using JointAcceleration_t = typename KUKA::Dynamics::JointAcceleration_t;
   using ExtLinkForces_t = typename KUKA::Dynamics::ExtLinkForces_t;

   JointState<KUKA::Kinematics::NJOINTS> irb_state;
   irb_state.setZero();

   control_vector_t torque_u = control_vector_t::Zero();
   ExtLinkForces_t ext_forces;
   ext_forces = ForceVector_t::Zero();
   JointAcceleration_t irb_xd;
   testdynamics.FixBaseForwardDynamics(irb_state, torque_u, ext_forces, irb_xd);
   JointAcceleration_t qdd;

   // id
   testdynamics.FixBaseID(irb_state, qdd, ext_forces, torque_u);

   // Kinematics, position EE
   KUKA::Kinematics kyn_;
   EndEffector<KUKA::Kinematics::NJOINTS> eeTest;
   for (size_t i = 0; i < KUKA::Kinematics::NUM_EE; i++) {
       eeTest = kyn_.getEndEffector(i);
   }
   JointState<KUKA::Kinematics::NJOINTS> kukaJointState;
   RigidBodyPose kukaPose;
   kukaJointState.setZero();
   kukaPose.setIdentity();
   size_t ind = 0;
   kindr::Position3D pos = kyn_.getEEPositionInWorld(ind, kukaPose, kukaJointState.getPositions());
   // std::cout << pos << std::endl;


   // Kinematics, velocity EE
   RigidBodyState baseState;
   baseState.setIdentity();
   Eigen::Matrix<double,2*KUKA::Kinematics::NJOINTS, 1>& state_ = kukaJointState.toImplementation();
   double qd[7] = {0,0,0,2,0,0,0}; 
   memcpy(state_.tail(7).data(), qd, 7*sizeof(double));
   // state_(10,0) = 2;
   // std::cout << kukaJointState.getVelocity() << std::endl;
   RBDState<KUKA::Kinematics::NJOINTS> RBD = RBDState<KUKA::Kinematics::NJOINTS>(baseState, kukaJointState);
   kindr::Velocity3D vel = kyn_.getEEVelocityInBase(ind, RBD);
   // std::cout << vel << std::endl;


   // Jacobian
   KUKA::Kinematics::Jacobian jac;
   size_t ee_id = 0;
   jac = kyn_.getJacobianBaseEEbyId(ee_id, RBD);

   std::cout << jac << std::endl;
}

void TEST_JACOBIAN() {

    typedef double valType;
    typedef KUKA::tpl::Kinematics<valType> Kinematics_t;
    typedef tpl::ConstraintJacobian<Kinematics_t, 6, 7, valType> ConstraintJacobian_t;

    ConstraintJacobian_t Jc_t;

    Jc_t.ee_indices_.push_back(0);
    Jc_t.eeInContact_[0] = true;
    
    ConstraintJacobian_t::jacobian_t jac, dJdt1, dJdt2, diff;
    KUKA::tpl::Dynamics<valType>::RBDState_t hyq_state;

    hyq_state.setZero();

    Jc_t.getJacobianOriginDerivative(hyq_state, dJdt1);
    Jc_t.getJacobianOriginDerivativeNumdiff(hyq_state, dJdt2);

    Jc_t.getJacobianOrigin(hyq_state, jac);

    std::cout << dJdt1 << std::endl;

    diff = dJdt1 - dJdt2;
    // auto maxError = diff.cwiseAbs().maxCoeff();
}

void TEST_OPERATIONAL_MODEL() {

    typedef Eigen::Matrix<double, 18, 1> robot_coordinate_t;
    typedef Eigen::Matrix<double, 3, 1> operational_coordinate_t;
    typedef typename OperationalModel<3, 1, 0>::state_t state_t;

    // pointer to the test robot RBDContainer
    std::shared_ptr<KUKA::RobCoGenContainer> testRbdContainerPtr(new KUKA::RobCoGenContainer());
    // end-effector array for the possible contact points
    std::array<EndEffector<7>, 0> EndEffectorArray;

    // // test robot model
    // OperationalModelRBD<KUKA::RobCoGenContainer, 0>::ptr testRobotModel(
    //     new OperationalModelRBD<KUKA::RobCoGenContainer, 0>(testRbdContainerPtr, EndEffectorArray));

    // test robot Jacobian
    // TestJacobian::ptr testJacobian(new TestJacobian());
    // // operational space
    // OperationalModel<3, 12, 0> testOperationalModel(testRobotModel, testJacobian);

    // Eigen::Matrix<double, 12, 1> tau = Eigen::Matrix<double, 12, 1>::Random();
    // state_t state;
    // state.fromStateVectorEulerXyz(state_t::state_vector_euler_t::Random());
    // //robot_coordinate_t q  = state.toCoordinatePosition();
    // //robot_coordinate_t qd = state.toCoordinateVelocity();

    // // update the robot model
    // testRobotModel->update(state);
    // robot_coordinate_t qdd;
    // qdd = testRobotModel->MInverse() *
    //       (-testRobotModel->C() - testRobotModel->G() + testRobotModel->S().transpose() * tau);

    // // update the operational space model
    // testOperationalModel.update(state);
    // operational_coordinate_t xdd = testOperationalModel.getAccelerations(qdd);

    // operational_coordinate_t testModelError = testOperationalModel.M() * xdd + testOperationalModel.C() +
    //                                           testOperationalModel.G() - testOperationalModel.S().transpose() * tau;

    // double maxError = testModelError.cwiseAbs().maxCoeff();

}

void TEST_AUTODIFF() {

  typedef KUKASoftContactFDSystem<KUKA::Dynamics> KUKASystem;

  const size_t STATE_DIM = KUKASystem::STATE_DIM;
  const size_t CONTROL_DIM = KUKASystem::CONTROL_DIM;

  ct::models::KUKA::KUKASoftContactSystemLinearizedForward kukaLinear;

  ct::core::StateVector<STATE_DIM> x;
  ct::core::ControlVector<CONTROL_DIM> u;


  x.setRandom();
  u.setRandom();

  auto A_gen = kukaLinear.getDerivativeState(x, u, 0.0);

  std::cout << A_gen << std::endl;

  // auto B_gen = kukaLinear.getDerivativeControl(x, u, 0.0);

  // std::cout << B_gen << std::endl;
  
}


int main() {

	// TEST_DYNAMICS();
	// TEST_OPERATIONAL_MODEL();
	// TEST_JACOBIAN();
	// TEST_RCG();
  TEST_AUTODIFF();
}


