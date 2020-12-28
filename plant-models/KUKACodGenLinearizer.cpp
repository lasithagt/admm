/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#include <ct/core/core.h>
#include <ct/optcon/optcon.h>
#include <ct/rbd/rbd.h>

#include <cmath>
#include <memory>

#include "RobCodGen/KUKA.h"
#include <ct/rbd/systems/FixBaseFDSystem.h>

#include <ct/models/CodegenOutputDirs.h>

const size_t state_dim = ct::rbd::FixBaseFDSystem<ct::rbd::KUKA::Dynamics>::STATE_DIM;
const size_t control_dim = ct::rbd::FixBaseFDSystem<ct::rbd::KUKA::Dynamics>::CONTROL_DIM;
const size_t njoints = ct::rbd::KUKA::Dynamics::NJOINTS;

typedef ct::core::ADCodegenLinearizer<state_dim, control_dim>::ADCGScalar Scalar;
typedef ct::rbd::FixBaseFDSystem<ct::rbd::KUKA::tpl::Dynamics<Scalar>> KUKANonLinearSystem;
typedef ct::core::DerivativesCppadCG<state_dim, control_dim> JacCG;

template <typename SCALAR>
using control_vector_t = typename ct::rbd::KUKA::tpl::Dynamics<SCALAR>::control_vector_t;

template <typename SCALAR>
using ExtLinkForces_t = typename ct::rbd::KUKA::tpl::Dynamics<SCALAR>::ExtLinkForces_t;

// Computes the torque needed to compensate gravity
template <typename SCALAR>
Eigen::Matrix<SCALAR, control_dim, 1> kukaInverseDynamics(const Eigen::Matrix<SCALAR, state_dim, 1>& x)
{
    ct::rbd::KUKA::tpl::Dynamics<SCALAR> kukaDynamics;
    ct::rbd::JointState<njoints, SCALAR> kukaState(x);
    Eigen::Matrix<SCALAR, njoints, 1> qddTmp = Eigen::Matrix<SCALAR, njoints, 1>::Zero();
    ct::rbd::JointAcceleration<njoints, SCALAR> qdd(qddTmp);             //zero
    ExtLinkForces_t<SCALAR> fext(Eigen::Matrix<SCALAR, njoints, 1>::Zero());  //zero
    control_vector_t<SCALAR> y;
    kukaDynamics.FixBaseID(kukaState, qdd, fext, y);
    return y;
}

int main(int argc, char** argv)
{
    std::shared_ptr<KUKANonLinearSystem> kuka(new KUKANonLinearSystem());
    ct::core::ADCodegenLinearizer<state_dim, control_dim> adLinearizer(kuka);
    // typename JacCG::FUN_TYPE_CG f = kukaInverseDynamics<CppAD::AD<CppAD::cg::CG<double>>>;
    // JacCG jacCG(f);

    try
    {
        // std::cout << "Generating Jacobian of Inverse Dynamics wrt state using reverse mode... " << std::endl;
        // jacCG.generateJacobianSource("KUKAInverseDynJacReverse", ct::models::KUKA_CODEGEN_OUTPUT_DIR,
        //     ct::core::CODEGEN_TEMPLATE_DIR, "models", "HyA", JacCG::Sparsity::Ones(), true);

        // std::cout << "Generating Hessian of Inverse Dynamics wrt state using reverse mode... " << std::endl;
        // jacCG.generateHessianSource("KUKAInverseDynHessian", ct::models::KUKA_CODEGEN_OUTPUT_DIR,
        //     ct::core::CODEGEN_TEMPLATE_DIR, "models", "HyA", JacCG::HessianSparsity::Ones(), true);

        std::cout << "generating using forward mode" << std::endl;
        adLinearizer.generateCode("KUKALinearizedForward", ct::models::KUKA_CODEGEN_OUTPUT_DIR,
            ct::core::CODEGEN_TEMPLATE_DIR, "models", "KUKA", false);

        std::cout << "generating using reverse mode" << std::endl;
        adLinearizer.generateCode("KUKALinearizedReverse", ct::models::KUKA_CODEGEN_OUTPUT_DIR,
            ct::core::CODEGEN_TEMPLATE_DIR, "models", "KUKA", true);

        // std::cout << "Generating Jacobian of Inverse Dynamics wrt state using forward mode... " << std::endl;
        // jacCG.generateJacobianSource("KUKAInverseDynJacForward", ct::models::KUKA_CODEGEN_OUTPUT_DIR,
        //     ct::core::CODEGEN_TEMPLATE_DIR, "models", "HyA", JacCG::Sparsity::Ones(), false);

        std::cout << "... done!" << std::endl;


    } catch (const std::runtime_error& e)
    {
        std::cout << "code generation failed: " << e.what() << std::endl;
    }

    return 0;
}
