#include "KukaModel.h"
#include "models.h"

#include "RobCodGenModel.h"


#include <ct/rbd/rbd.h>

// #include "testIrb4600/RobCoGenTestIrb4600.h"
#include "RobCodGen/KUKA.h"
#include "ct/rbd/robot/Dynamics.h"
#include "ct/rbd/robot/Kinematics.h"

using namespace ct::rbd;


// Test scripts
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");


int main() {


	std::shared_ptr<KUKA::Kinematics> kyn(new KUKA::Kinematics);

    KUKA::Dynamics testdynamics(kyn);

    using control_vector_t = typename KUKA::Dynamics::control_vector_t;
    using ForceVector_t = typename KUKA::Dynamics::ForceVector_t;
    using JointState_t = typename KUKA::Dynamics::JointState_t;
    using JointAcceleration_t = typename KUKA::Dynamics::JointAcceleration_t;
    using ExtLinkForces_t = typename KUKA::Dynamics::ExtLinkForces_t;

    JointState_t irb_state;
    irb_state.setZero();

    control_vector_t torque_u = control_vector_t::Zero();
    ExtLinkForces_t ext_forces;
    ext_forces = ForceVector_t::Zero();

    JointAcceleration_t irb_xd;

    testdynamics.FixBaseForwardDynamics(irb_state, torque_u, ext_forces, irb_xd);

    JointAcceleration_t qdd;
    testdynamics.FixBaseID(irb_state, qdd, ext_forces, torque_u);

    



    // Kinematics
    KUKA::Kinematics kyn_;
    EndEffector<KUKA::Kinematics::NJOINTS> eeTest;

    for (size_t i = 0; i < KUKA::Kinematics::NUM_EE; i++)
    {
        eeTest = kyn_.getEndEffector(i);
    }

    JointState<KUKA::Kinematics::NJOINTS> kukaJointState;

    RigidBodyPose kukaPose;

    kukaJointState.setZero();
    kukaPose.setIdentity();

    size_t ind = 0;

    kindr::Position3D pos = kyn_.getEEPositionInWorld(ind, kukaPose, kukaJointState.getPositions());

    std::cout << pos << std::endl;


}


