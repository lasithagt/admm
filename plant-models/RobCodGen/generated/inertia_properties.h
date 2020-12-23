#ifndef IIT_ROBOT_KUKA_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_KUKA_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"

namespace iit {
namespace Kuka {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot Kuka.
 */
namespace dyn {

using InertiaMatrix = iit::rbd::InertiaMatrixDense;

namespace tpl {

template <typename TRAIT>
class InertiaProperties {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef typename TRAIT::Scalar Scalar;
        typedef iit::rbd::Core<Scalar> CoreS;
        typedef iit::rbd::tpl::InertiaMatrixDense<Scalar> IMatrix;
        typedef typename CoreS::Vector3 Vec3d;

        InertiaProperties();
        ~InertiaProperties();
        const IMatrix& getTensor_link1() const;
        const IMatrix& getTensor_link2() const;
        const IMatrix& getTensor_link3() const;
        const IMatrix& getTensor_link4() const;
        const IMatrix& getTensor_link5() const;
        const IMatrix& getTensor_link6() const;
        const IMatrix& getTensor_link7() const;
        Scalar getMass_link1() const;
        Scalar getMass_link2() const;
        Scalar getMass_link3() const;
        Scalar getMass_link4() const;
        Scalar getMass_link5() const;
        Scalar getMass_link6() const;
        Scalar getMass_link7() const;
        const Vec3d& getCOM_link1() const;
        const Vec3d& getCOM_link2() const;
        const Vec3d& getCOM_link3() const;
        const Vec3d& getCOM_link4() const;
        const Vec3d& getCOM_link5() const;
        const Vec3d& getCOM_link6() const;
        const Vec3d& getCOM_link7() const;
        Scalar getTotalMass() const;

    private:

        IMatrix tensor_link1;
        IMatrix tensor_link2;
        IMatrix tensor_link3;
        IMatrix tensor_link4;
        IMatrix tensor_link5;
        IMatrix tensor_link6;
        IMatrix tensor_link7;
        Vec3d com_link1;
        Vec3d com_link2;
        Vec3d com_link3;
        Vec3d com_link4;
        Vec3d com_link5;
        Vec3d com_link6;
        Vec3d com_link7;
};

template <typename TRAIT>
inline InertiaProperties<TRAIT>::~InertiaProperties() {}

template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link1() const {
    return this->tensor_link1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link2() const {
    return this->tensor_link2;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link3() const {
    return this->tensor_link3;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link4() const {
    return this->tensor_link4;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link5() const {
    return this->tensor_link5;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link6() const {
    return this->tensor_link6;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_link7() const {
    return this->tensor_link7;
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_link1() const {
    return this->tensor_link1.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_link2() const {
    return this->tensor_link2.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_link3() const {
    return this->tensor_link3.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_link4() const {
    return this->tensor_link4.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_link5() const {
    return this->tensor_link5.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_link6() const {
    return this->tensor_link6.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_link7() const {
    return this->tensor_link7.getMass();
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_link1() const {
    return this->com_link1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_link2() const {
    return this->com_link2;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_link3() const {
    return this->com_link3;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_link4() const {
    return this->com_link4;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_link5() const {
    return this->com_link5;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_link6() const {
    return this->com_link6;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_link7() const {
    return this->com_link7;
}

template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getTotalMass() const {
    return 4.0 + 4.0 + 3.0 + 2.7 + 1.7 + 1.8 + 0.3;
}

}

using InertiaProperties = tpl::InertiaProperties<rbd::DoubleTrait>;

}
}
}

#include "inertia_properties.impl.h"

#endif
