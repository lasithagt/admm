/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/systems/FixBaseSystemBase.h>
#include <ct/rbd/robot/actuator/ActuatorDynamics.h>
#include <ct/rbd/state/FixBaseRobotState.h>
#include "SoftContactModel.h"

namespace ct {
namespace rbd {

/**
 * \brief A fix base rigid body system that uses forward dynamics.
 *
 * A fix base rigid body system that uses forward dynamics. The control input vector is assumed to consist of
 *  - joint torques and
 *  - end effector forces expressed in the world frame
 *
 * The overall state vector is arranged in the order
 * - joint positions
 * - joint velocities
 * - force vector
 * - actuator state
 *
 * \warning when modelled with RobCoGen, the base pose must be rotated "against gravity" (RobCoGen modeling assumption)
 */
template <class RBDDynamics, size_t ACT_STATE_DIM = 0, bool EE_ARE_CONTROL_INPUTS = false>
class KUKASoftContactFDSystem : public FixBaseSystemBase<RBDDynamics,
                            RBDDynamics::NSTATE + ACT_STATE_DIM + 3,                                   // state dim
                            RBDDynamics::NJOINTS + EE_ARE_CONTROL_INPUTS * RBDDynamics::N_EE * 3>  // control dim
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const size_t N_EE = RBDDynamics::N_EE; 
    static const size_t FORCE_DIM = 3;                        //! number of end-effectors
    static const size_t STATE_DIM = RBDDynamics::NSTATE + ACT_STATE_DIM + FORCE_DIM;  // combined state dim
    static const size_t CONTROL_DIM = RBDDynamics::NJOINTS + EE_ARE_CONTROL_INPUTS * N_EE * 3;  // combined control dim
    static const size_t ACTUATOR_STATE_DIM = ACT_STATE_DIM;


    using Kinematics = typename RBDDynamics::Kinematics_t;
    using BASE = FixBaseSystemBase<RBDDynamics, STATE_DIM, CONTROL_DIM>;
    using SCALAR = typename BASE::SCALAR;
    using state_vector_t = typename BASE::state_vector_t;
    using control_vector_t = typename BASE::control_vector_t;
    // using JointAcceleration_t = Eigen::Matrix<SCALAR, RBDDynamics::NSTATE + FORCE_DIM, 1>;
    using force_vector_t = typename Eigen::Matrix<SCALAR, 3, 1>;
    using ForceAcceleration_t = force_vector_t;
    using JointAcceleration_t = typename BASE::JointAcceleration_t;

    using RigidBodyPose_t = typename BASE::RigidBodyPose_t;
    // using Jacobian_t = typename KUKA::Jacobian_t;

    using SoftContactModel_t = ContactModel::SoftContactModel<SCALAR>;
    using ActuatorDynamics_t = ActuatorDynamics<ACTUATOR_STATE_DIM, RBDDynamics::NJOINTS, SCALAR>;
    using FixBaseRobotState_t = FixBaseRobotState<BASE::NJOINTS, ACTUATOR_STATE_DIM, SCALAR>;
    using actuator_state_vector_t = typename FixBaseRobotState_t::actuator_state_vector_t;
    using vector3_t = typename SoftContactModel_t::Vector3s;
    using matrix3_t = typename SoftContactModel_t::Matrix3s;


    /*!
     * @brief constructor
     * \warning when using actuator dynamics, the system looses its second order characteristics
     */
    KUKASoftContactFDSystem(const RigidBodyPose_t& basePose = RigidBodyPose_t()) : BASE(basePose), actuatorDynamics_(nullptr) {

        // contact model initilizatioon
        ContactModel::ContactParams<SCALAR> cp_;
        cp_.E = SCALAR(1000);
        cp_.mu = SCALAR(0.5);
        cp_.nu = SCALAR(0.4);
        cp_.R  = SCALAR(0.005);
        cp_.R_path = SCALAR(1000);
        cp_.Kd = SCALAR(10);
        contactModel_ = std::shared_ptr<SoftContactModel_t>(new SoftContactModel_t(cp_));
    }
    /*!
     * @brief constructor including actuator dynamics
     * \warning when using actuator dynamics, the system looses its second order characteristics
     */
    KUKASoftContactFDSystem(std::shared_ptr<ActuatorDynamics_t> actuatorDynamics,
        const RigidBodyPose_t& basePose = RigidBodyPose_t())
        : BASE(basePose), actuatorDynamics_(actuatorDynamics)
    {
    }

    /*!
     * @brief copy constructor
	 *
	 * @param arg instance of FixBaseFDSystem to be copied.
	 *
	 * \note takes care of explicitly cloning actuatorDynamics, if existent
	 */
    KUKASoftContactFDSystem(const KUKASoftContactFDSystem& arg) : BASE(arg)
    {
        if (arg.actuatorDynamics_)
        {
            actuatorDynamics_ = std::shared_ptr<ActuatorDynamics_t>(arg.actuatorDynamics_->clone());
        }

    }

    //! destructor
    virtual ~KUKASoftContactFDSystem() = default;

    //! compute the controlled dynamics of the fixed base robotic system
    void computeControlledDynamics(const state_vector_t& state,
        const SCALAR& t,
        const control_vector_t& controlIn,
        state_vector_t& derivative) override
    {
        FixBaseRobotState_t robotState(state.template topRows<2 * BASE::NJOINTS>());

        // map the joint velocities (unchanged, no damping)
        derivative.template topRows<BASE::NJOINTS>() = robotState.joints().getVelocities();

        // temporary variable for the control (will get modified by the actuator dynamics, if applicable)
        control_vector_t control = controlIn;

        // compute actuator dynamics and their control output
        computeActuatorDynamics(robotState, t, controlIn, derivative, control);

        // Cache updated rbd state
        typename RBDDynamics::ExtLinkForces_t linkForces(Eigen::Matrix<SCALAR, 6, 1>::Zero());


        // for (size_t i = 0; i < RBDDynamics::N_EE; i++)
        // {
        //     auto endEffector = this->dynamics_.kinematics().getEndEffector(i);
        //     size_t linkId = endEffector.getLinkId();
        //     linkForces[static_cast<typename RBDDynamics::ROBCOGEN::LinkIdentifiers>(linkId)] =
        //         this->dynamics_.kinematics().mapForceFromWorldToLink3d(
        //             state.template tail<3>(), this->basePose_,
        //             robotState.joints().getPositions(), i);
        // }

        // add end effector forces as control inputs (if applicable)
        if (EE_ARE_CONTROL_INPUTS == true)
        {
            for (size_t i = 0; i < RBDDynamics::N_EE; i++)
            {
                auto endEffector = this->dynamics_.kinematics().getEndEffector(i);
                size_t linkId = endEffector.getLinkId();
                linkForces[static_cast<typename RBDDynamics::ROBCOGEN::LinkIdentifiers>(linkId)] =
                    this->dynamics_.kinematics().mapForceFromWorldToLink3d(
                        control.template segment<3>(BASE::NJOINTS + i * 3), this->basePose_,
                        robotState.joints().getPositions(), i);
            }
        }

        typename RBDDynamics::JointAcceleration_t jAcc;
        // typename RBDDynamics::JointAcceleration_t jAcc_id;
        control_vector_t torque_grav;
        // jAcc_id.setZero();

        // compute robot dynamics 
        Eigen::Matrix<SCALAR, 2*BASE::NJOINTS, 1> robotState_id(state.template topRows<2 * BASE::NJOINTS>());
        Eigen::Matrix<SCALAR, BASE::NJOINTS, 1> jAcc_id;
        jAcc_id.setZero();
        robotState_id.template segment<7>(7).setZero();

        this->dynamics_.FixBaseID(robotState_id, jAcc_id, torque_grav);
        control += torque_grav;

        this->dynamics_.FixBaseForwardDynamics(
            robotState.joints(), control.template head<BASE::NJOINTS>(), linkForces, jAcc);

        // jAcc.setZero();
        

        // compute kinematics, position, velocity and acceleration of the ee
        // Kinematics, position
        size_t ee_id = 0;
        kindr::Position<SCALAR, 3> pose_EE_pos = this->dynamics_.kinematicsPtr()->getEEPositionInWorld(ee_id, this->basePose_, robotState.joints().getPositions());
        Eigen::Matrix<SCALAR, 3, 3> pose_EE_rot = this->dynamics_.kinematicsPtr()->getEERotInWorld(ee_id, this->basePose_, robotState.joints().getPositions());

        
        // Kinematics, velocity
        Eigen::Matrix<SCALAR, 3, 1> velocity_ee = this->dynamics_.kinematicsPtr()->getEEVelocityInBase(ee_id, robotState.toRBDState()).vector();
        Eigen::Matrix<SCALAR, 6, 7> jac = this->dynamics_.kinematicsPtr()->getJacobianBaseEEbyId(ee_id, robotState.toRBDState());
        Eigen::Matrix<SCALAR, 3, 1> accel_ee = jac.template bottomRows<3>() * jAcc.getAcceleration();

        Eigen::Matrix<SCALAR, 3, RBDDynamics::NJOINTS> Jc_Rotational, Jc_Translational, dJdt;
        Eigen::Matrix<SCALAR, 3, 1> dJidqj;
        dJdt.setZero();

        // Collect current contact Jacobians
        Jc_Rotational = jac.template topRows<3>();
        Jc_Translational = jac.template bottomRows<3>();

        // Compute dJdt for the joint columns
        for (size_t i = 0; i < RBDDynamics::NJOINTS; i++)
        {  // Loop over columns i of dJdt_joints
         for (size_t j = 0; j < RBDDynamics::NJOINTS; j++)
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
             dJdt.template block<3, 1>(0, i) += dJidqj * robotState.joints().getVelocities()(j);
         }
        }
        // Fill in the rows of the full dJdt
        accel_ee += dJdt * robotState.joints().getVelocities();

        // compute the cartesian space mass matrix
        Eigen::Matrix<SCALAR, 3, 3> massMatrix;
        vector3_t df_;
        // matrix3_t orientation;


        // // compute contact dynamics 
        contactModel_->df(massMatrix, pose_EE_pos.vector(), pose_EE_rot, velocity_ee, accel_ee, state.template tail<3>(), df_);


        // combine manipulator and contact dynamics
        derivative.template segment<BASE::NJOINTS>(BASE::NJOINTS) = jAcc.getAcceleration();
        derivative.template tail<3>() = df_;
    }




    //! evaluate the actuator dynamics if actuators are enabled
    template <typename T = void>
    typename std::enable_if<(ACT_STATE_DIM > 0), T>::type computeActuatorDynamics(const FixBaseRobotState_t& robotState,
        const SCALAR& t,
        const control_vector_t& controlIn,
        state_vector_t& stateDerivative,
        control_vector_t& controlOut)
    {
        // get references to the current actuator position, velocity and input
        const Eigen::Ref<const typename control_vector_t::Base> actControlIn =
            controlIn.template topRows<BASE::NJOINTS>();

        actuator_state_vector_t actStateDerivative;  // todo use vector block for this?
        actuatorDynamics_->computeActuatorDynamics(
            robotState.joints(), robotState.actuatorState(), t, actControlIn, actStateDerivative);

        stateDerivative.template tail<ACTUATOR_STATE_DIM>() = actStateDerivative;

        // overwrite control with actuator control output as a function of current robot and actuator states
        controlOut = actuatorDynamics_->computeControlOutput(robotState.joints(), robotState.actuatorState());
    }

    //! do nothing if actuators disabled
    template <typename T = void>
    typename std::enable_if<(ACT_STATE_DIM == 0), T>::type computeActuatorDynamics(
        const FixBaseRobotState_t& robotState,
        const SCALAR& t,
        const control_vector_t& controlIn,
        state_vector_t& stateDerivative,
        control_vector_t& controlOut)
    {
    }

    //! deep cloning
    virtual KUKASoftContactFDSystem<RBDDynamics, ACT_STATE_DIM, EE_ARE_CONTROL_INPUTS>* clone() const override
    {
        return new KUKASoftContactFDSystem<RBDDynamics, ACT_STATE_DIM, EE_ARE_CONTROL_INPUTS>(*this);
    }

    //! get pointer to actuator dynamics
    std::shared_ptr<ActuatorDynamics_t> getActuatorDynamics() { return actuatorDynamics_; }
    //! if actuator dynamics enabled, this method allows to design a consistent actuator state
    template <typename T = typename FixBaseRobotState_t::actuator_state_vector_t>
    typename std::enable_if<(ACT_STATE_DIM > 0), T>::type computeConsistentActuatorState(
        const JointState<BASE::NJOINTS, SCALAR>& jStateRef,
        const ct::core::ControlVector<BASE::NJOINTS>& torqueRef)
    {
        return actuatorDynamics_->computeStateFromOutput(jStateRef, torqueRef);
    }

    //! if actuator dynamics enabled, this method allows to design a consistent actuator state
    template <typename T = typename FixBaseRobotState_t::actuator_state_vector_t>
    typename std::enable_if<(ACT_STATE_DIM > 0), T>::type computeConsistentActuatorState(
        const JointState<BASE::NJOINTS, SCALAR>& jStateRef)
    {
        const ct::core::ControlVector<BASE::NJOINTS> torqueRef = computeIDTorques(jStateRef);
        return computeConsistentActuatorState(jStateRef, torqueRef);
    }


private:
    //! pointer to the actuator dynamics
    std::shared_ptr<ActuatorDynamics_t> actuatorDynamics_;
    std::shared_ptr<SoftContactModel_t> contactModel_;
    std::shared_ptr<Kinematics> kinematics_;

};

}  // namespace rbd
}  // namespace ct
