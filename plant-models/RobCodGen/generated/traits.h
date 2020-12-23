#ifndef IIT_ROBOGEN__KUKA_TRAITS_H_
#define IIT_ROBOGEN__KUKA_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace Kuka {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename Kuka::JointIdentifiers JointID;
    typedef typename Kuka::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename Kuka::tpl::JointState<SCALAR> JointState;



    typedef typename Kuka::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename Kuka::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename Kuka::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename Kuka::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::Kuka::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::Kuka::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::Kuka::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::Kuka::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = Kuka::jointsCount;
    static const int links_count  = Kuka::linksCount;
    static const bool floating_base = false;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return Kuka::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return Kuka::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
