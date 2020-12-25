
// Initialization of static-const data
template <typename TRAIT>
const typename iit::Kuka::dyn::tpl::ForwardDynamics<TRAIT>::ExtForces
    iit::Kuka::dyn::tpl::ForwardDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::Kuka::dyn::tpl::ForwardDynamics<TRAIT>::ForwardDynamics(iit::Kuka::dyn::tpl::InertiaProperties<TRAIT>& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    link1_v.setZero();
    link1_c.setZero();
    link2_v.setZero();
    link2_c.setZero();
    link3_v.setZero();
    link3_c.setZero();
    link4_v.setZero();
    link4_c.setZero();
    link5_v.setZero();
    link5_c.setZero();
    link6_v.setZero();
    link6_c.setZero();
    link7_v.setZero();
    link7_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

template <typename TRAIT>
void iit::Kuka::dyn::tpl::ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    link1_AI = inertiaProps->getTensor_link1();
    link1_p = - fext[LINK1];
    link2_AI = inertiaProps->getTensor_link2();
    link2_p = - fext[LINK2];
    link3_AI = inertiaProps->getTensor_link3();
    link3_p = - fext[LINK3];
    link4_AI = inertiaProps->getTensor_link4();
    link4_p = - fext[LINK4];
    link5_AI = inertiaProps->getTensor_link5();
    link5_p = - fext[LINK5];
    link6_AI = inertiaProps->getTensor_link6();
    link6_p = - fext[LINK6];
    link7_AI = inertiaProps->getTensor_link7();
    link7_p = - fext[LINK7];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link link1
    //  - The spatial velocity:
    link1_v(iit::rbd::AZ) = qd(JA);
    
    //  - The bias force term:
    link1_p += iit::rbd::vxIv(qd(JA), link1_AI);
    
    // + Link link2
    //  - The spatial velocity:
    link2_v = (motionTransforms-> fr_link2_X_fr_link1) * link1_v;
    link2_v(iit::rbd::AZ) += qd(JB);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(link2_v, vcross);
    link2_c = vcross.col(iit::rbd::AZ) * qd(JB);
    
    //  - The bias force term:
    link2_p += iit::rbd::vxIv(link2_v, link2_AI);
    
    // + Link link3
    //  - The spatial velocity:
    link3_v = (motionTransforms-> fr_link3_X_fr_link2) * link2_v;
    link3_v(iit::rbd::AZ) += qd(JC);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(link3_v, vcross);
    link3_c = vcross.col(iit::rbd::AZ) * qd(JC);
    
    //  - The bias force term:
    link3_p += iit::rbd::vxIv(link3_v, link3_AI);
    
    // + Link link4
    //  - The spatial velocity:
    link4_v = (motionTransforms-> fr_link4_X_fr_link3) * link3_v;
    link4_v(iit::rbd::AZ) += qd(JD);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(link4_v, vcross);
    link4_c = vcross.col(iit::rbd::AZ) * qd(JD);
    
    //  - The bias force term:
    link4_p += iit::rbd::vxIv(link4_v, link4_AI);
    
    // + Link link5
    //  - The spatial velocity:
    link5_v = (motionTransforms-> fr_link5_X_fr_link4) * link4_v;
    link5_v(iit::rbd::AZ) += qd(JE);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(link5_v, vcross);
    link5_c = vcross.col(iit::rbd::AZ) * qd(JE);
    
    //  - The bias force term:
    link5_p += iit::rbd::vxIv(link5_v, link5_AI);
    
    // + Link link6
    //  - The spatial velocity:
    link6_v = (motionTransforms-> fr_link6_X_fr_link5) * link5_v;
    link6_v(iit::rbd::AZ) += qd(JF);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(link6_v, vcross);
    link6_c = vcross.col(iit::rbd::AZ) * qd(JF);
    
    //  - The bias force term:
    link6_p += iit::rbd::vxIv(link6_v, link6_AI);
    
    // + Link link7
    //  - The spatial velocity:
    link7_v = (motionTransforms-> fr_link7_X_fr_link6) * link6_v;
    link7_v(iit::rbd::AZ) += qd(JG);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(link7_v, vcross);
    link7_c = vcross.col(iit::rbd::AZ) * qd(JG);
    
    //  - The bias force term:
    link7_p += iit::rbd::vxIv(link7_v, link7_AI);
    
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66S IaB;
    Force pa;
    
    // + Link link7
    link7_u = tau(JG) - link7_p(iit::rbd::AZ);
    link7_U = link7_AI.col(iit::rbd::AZ);
    link7_D = link7_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(link7_AI, link7_U, link7_D, Ia_r);  // same as: Ia_r = link7_AI - link7_U/link7_D * link7_U.transpose();
    pa = link7_p + Ia_r * link7_c + link7_U * link7_u/link7_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_link7_X_fr_link6, IaB);
    link6_AI += IaB;
    link6_p += (motionTransforms-> fr_link7_X_fr_link6).transpose() * pa;
    
    // + Link link6
    link6_u = tau(JF) - link6_p(iit::rbd::AZ);
    link6_U = link6_AI.col(iit::rbd::AZ);
    link6_D = link6_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(link6_AI, link6_U, link6_D, Ia_r);  // same as: Ia_r = link6_AI - link6_U/link6_D * link6_U.transpose();
    pa = link6_p + Ia_r * link6_c + link6_U * link6_u/link6_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_link6_X_fr_link5, IaB);
    link5_AI += IaB;
    link5_p += (motionTransforms-> fr_link6_X_fr_link5).transpose() * pa;
    
    // + Link link5
    link5_u = tau(JE) - link5_p(iit::rbd::AZ);
    link5_U = link5_AI.col(iit::rbd::AZ);
    link5_D = link5_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(link5_AI, link5_U, link5_D, Ia_r);  // same as: Ia_r = link5_AI - link5_U/link5_D * link5_U.transpose();
    pa = link5_p + Ia_r * link5_c + link5_U * link5_u/link5_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_link5_X_fr_link4, IaB);
    link4_AI += IaB;
    link4_p += (motionTransforms-> fr_link5_X_fr_link4).transpose() * pa;
    
    // + Link link4
    link4_u = tau(JD) - link4_p(iit::rbd::AZ);
    link4_U = link4_AI.col(iit::rbd::AZ);
    link4_D = link4_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(link4_AI, link4_U, link4_D, Ia_r);  // same as: Ia_r = link4_AI - link4_U/link4_D * link4_U.transpose();
    pa = link4_p + Ia_r * link4_c + link4_U * link4_u/link4_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_link4_X_fr_link3, IaB);
    link3_AI += IaB;
    link3_p += (motionTransforms-> fr_link4_X_fr_link3).transpose() * pa;
    
    // + Link link3
    link3_u = tau(JC) - link3_p(iit::rbd::AZ);
    link3_U = link3_AI.col(iit::rbd::AZ);
    link3_D = link3_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(link3_AI, link3_U, link3_D, Ia_r);  // same as: Ia_r = link3_AI - link3_U/link3_D * link3_U.transpose();
    pa = link3_p + Ia_r * link3_c + link3_U * link3_u/link3_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_link3_X_fr_link2, IaB);
    link2_AI += IaB;
    link2_p += (motionTransforms-> fr_link3_X_fr_link2).transpose() * pa;
    
    // + Link link2
    link2_u = tau(JB) - link2_p(iit::rbd::AZ);
    link2_U = link2_AI.col(iit::rbd::AZ);
    link2_D = link2_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(link2_AI, link2_U, link2_D, Ia_r);  // same as: Ia_r = link2_AI - link2_U/link2_D * link2_U.transpose();
    pa = link2_p + Ia_r * link2_c + link2_U * link2_u/link2_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_link2_X_fr_link1, IaB);
    link1_AI += IaB;
    link1_p += (motionTransforms-> fr_link2_X_fr_link1).transpose() * pa;
    
    // + Link link1
    link1_u = tau(JA) - link1_p(iit::rbd::AZ);
    link1_U = link1_AI.col(iit::rbd::AZ);
    link1_D = link1_U(iit::rbd::AZ);
    
    
    
    // ---------------------- THIRD PASS ---------------------- //
    link1_a = (motionTransforms-> fr_link1_X_fr_base0).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    qdd(JA) = (link1_u - link1_U.dot(link1_a)) / link1_D;
    link1_a(iit::rbd::AZ) += qdd(JA);
    
    link2_a = (motionTransforms-> fr_link2_X_fr_link1) * link1_a + link2_c;
    qdd(JB) = (link2_u - link2_U.dot(link2_a)) / link2_D;
    link2_a(iit::rbd::AZ) += qdd(JB);
    
    link3_a = (motionTransforms-> fr_link3_X_fr_link2) * link2_a + link3_c;
    qdd(JC) = (link3_u - link3_U.dot(link3_a)) / link3_D;
    link3_a(iit::rbd::AZ) += qdd(JC);
    
    link4_a = (motionTransforms-> fr_link4_X_fr_link3) * link3_a + link4_c;
    qdd(JD) = (link4_u - link4_U.dot(link4_a)) / link4_D;
    link4_a(iit::rbd::AZ) += qdd(JD);
    
    link5_a = (motionTransforms-> fr_link5_X_fr_link4) * link4_a + link5_c;
    qdd(JE) = (link5_u - link5_U.dot(link5_a)) / link5_D;
    link5_a(iit::rbd::AZ) += qdd(JE);
    
    link6_a = (motionTransforms-> fr_link6_X_fr_link5) * link5_a + link6_c;
    qdd(JF) = (link6_u - link6_U.dot(link6_a)) / link6_D;
    link6_a(iit::rbd::AZ) += qdd(JF);
    
    link7_a = (motionTransforms-> fr_link7_X_fr_link6) * link6_a + link7_c;
    qdd(JG) = (link7_u - link7_U.dot(link7_a)) / link7_D;
    link7_a(iit::rbd::AZ) += qdd(JG);
    
    
}
