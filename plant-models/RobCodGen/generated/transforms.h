#ifndef KUKA_TRANSFORMS_H_
#define KUKA_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include <iit/rbd/traits/DoubleTrait.h>
#include "kinematics_parameters.h"

namespace iit {
namespace Kuka {

template<typename SCALAR, class M>
class TransformMotion : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformForce : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformHomogeneous : public iit::rbd::HomogeneousTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

namespace tpl {


/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
template <typename TRAIT>
class MotionTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformMotion<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_base0_X_fr_link1 : public TransformMotion<Scalar, Type_fr_base0_X_fr_link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link1();
        const Type_fr_base0_X_fr_link1& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link2 : public TransformMotion<Scalar, Type_fr_base0_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link2();
        const Type_fr_base0_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link3 : public TransformMotion<Scalar, Type_fr_base0_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link3();
        const Type_fr_base0_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link4 : public TransformMotion<Scalar, Type_fr_base0_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link4();
        const Type_fr_base0_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link5 : public TransformMotion<Scalar, Type_fr_base0_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link5();
        const Type_fr_base0_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link6 : public TransformMotion<Scalar, Type_fr_base0_X_fr_link6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link6();
        const Type_fr_base0_X_fr_link6& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link7 : public TransformMotion<Scalar, Type_fr_base0_X_fr_link7>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link7();
        const Type_fr_base0_X_fr_link7& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_ee : public TransformMotion<Scalar, Type_fr_base0_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_ee();
        const Type_fr_base0_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_link1_X_fr_base0 : public TransformMotion<Scalar, Type_fr_link1_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link1_X_fr_base0();
        const Type_fr_link1_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_base0 : public TransformMotion<Scalar, Type_fr_link2_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_base0();
        const Type_fr_link2_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link3_X_fr_base0 : public TransformMotion<Scalar, Type_fr_link3_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_base0();
        const Type_fr_link3_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_base0 : public TransformMotion<Scalar, Type_fr_link4_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_base0();
        const Type_fr_link4_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_base0 : public TransformMotion<Scalar, Type_fr_link5_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_base0();
        const Type_fr_link5_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link6_X_fr_base0 : public TransformMotion<Scalar, Type_fr_link6_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link6_X_fr_base0();
        const Type_fr_link6_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link7_X_fr_base0 : public TransformMotion<Scalar, Type_fr_link7_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link7_X_fr_base0();
        const Type_fr_link7_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_X_fr_base0 : public TransformMotion<Scalar, Type_fr_ee_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_X_fr_base0();
        const Type_fr_ee_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jA : public TransformMotion<Scalar, Type_fr_base0_X_fr_jA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jA();
        const Type_fr_base0_X_fr_jA& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jB : public TransformMotion<Scalar, Type_fr_base0_X_fr_jB>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jB();
        const Type_fr_base0_X_fr_jB& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jC : public TransformMotion<Scalar, Type_fr_base0_X_fr_jC>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jC();
        const Type_fr_base0_X_fr_jC& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jD : public TransformMotion<Scalar, Type_fr_base0_X_fr_jD>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jD();
        const Type_fr_base0_X_fr_jD& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jE : public TransformMotion<Scalar, Type_fr_base0_X_fr_jE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jE();
        const Type_fr_base0_X_fr_jE& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jF : public TransformMotion<Scalar, Type_fr_base0_X_fr_jF>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jF();
        const Type_fr_base0_X_fr_jF& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jG : public TransformMotion<Scalar, Type_fr_base0_X_fr_jG>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jG();
        const Type_fr_base0_X_fr_jG& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_link1 : public TransformMotion<Scalar, Type_fr_link2_X_fr_link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_link1();
        const Type_fr_link2_X_fr_link1& update(const JState&);
    protected:
    };
    
    class Type_fr_link1_X_fr_link2 : public TransformMotion<Scalar, Type_fr_link1_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link1_X_fr_link2();
        const Type_fr_link1_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_link3_X_fr_link2 : public TransformMotion<Scalar, Type_fr_link3_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_link2();
        const Type_fr_link3_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_link3 : public TransformMotion<Scalar, Type_fr_link2_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_link3();
        const Type_fr_link2_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_link3 : public TransformMotion<Scalar, Type_fr_link4_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_link3();
        const Type_fr_link4_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_link3_X_fr_link4 : public TransformMotion<Scalar, Type_fr_link3_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_link4();
        const Type_fr_link3_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_link4 : public TransformMotion<Scalar, Type_fr_link5_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_link4();
        const Type_fr_link5_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_link5 : public TransformMotion<Scalar, Type_fr_link4_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_link5();
        const Type_fr_link4_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_link6_X_fr_link5 : public TransformMotion<Scalar, Type_fr_link6_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link6_X_fr_link5();
        const Type_fr_link6_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_link6 : public TransformMotion<Scalar, Type_fr_link5_X_fr_link6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_link6();
        const Type_fr_link5_X_fr_link6& update(const JState&);
    protected:
    };
    
    class Type_fr_link7_X_fr_link6 : public TransformMotion<Scalar, Type_fr_link7_X_fr_link6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link7_X_fr_link6();
        const Type_fr_link7_X_fr_link6& update(const JState&);
    protected:
    };
    
    class Type_fr_link6_X_fr_link7 : public TransformMotion<Scalar, Type_fr_link6_X_fr_link7>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link6_X_fr_link7();
        const Type_fr_link6_X_fr_link7& update(const JState&);
    protected:
    };
    
public:
    MotionTransforms();
    void updateParameters();
    Type_fr_base0_X_fr_link1 fr_base0_X_fr_link1;
    Type_fr_base0_X_fr_link2 fr_base0_X_fr_link2;
    Type_fr_base0_X_fr_link3 fr_base0_X_fr_link3;
    Type_fr_base0_X_fr_link4 fr_base0_X_fr_link4;
    Type_fr_base0_X_fr_link5 fr_base0_X_fr_link5;
    Type_fr_base0_X_fr_link6 fr_base0_X_fr_link6;
    Type_fr_base0_X_fr_link7 fr_base0_X_fr_link7;
    Type_fr_base0_X_fr_ee fr_base0_X_fr_ee;
    Type_fr_link1_X_fr_base0 fr_link1_X_fr_base0;
    Type_fr_link2_X_fr_base0 fr_link2_X_fr_base0;
    Type_fr_link3_X_fr_base0 fr_link3_X_fr_base0;
    Type_fr_link4_X_fr_base0 fr_link4_X_fr_base0;
    Type_fr_link5_X_fr_base0 fr_link5_X_fr_base0;
    Type_fr_link6_X_fr_base0 fr_link6_X_fr_base0;
    Type_fr_link7_X_fr_base0 fr_link7_X_fr_base0;
    Type_fr_ee_X_fr_base0 fr_ee_X_fr_base0;
    Type_fr_base0_X_fr_jA fr_base0_X_fr_jA;
    Type_fr_base0_X_fr_jB fr_base0_X_fr_jB;
    Type_fr_base0_X_fr_jC fr_base0_X_fr_jC;
    Type_fr_base0_X_fr_jD fr_base0_X_fr_jD;
    Type_fr_base0_X_fr_jE fr_base0_X_fr_jE;
    Type_fr_base0_X_fr_jF fr_base0_X_fr_jF;
    Type_fr_base0_X_fr_jG fr_base0_X_fr_jG;
    Type_fr_link2_X_fr_link1 fr_link2_X_fr_link1;
    Type_fr_link1_X_fr_link2 fr_link1_X_fr_link2;
    Type_fr_link3_X_fr_link2 fr_link3_X_fr_link2;
    Type_fr_link2_X_fr_link3 fr_link2_X_fr_link3;
    Type_fr_link4_X_fr_link3 fr_link4_X_fr_link3;
    Type_fr_link3_X_fr_link4 fr_link3_X_fr_link4;
    Type_fr_link5_X_fr_link4 fr_link5_X_fr_link4;
    Type_fr_link4_X_fr_link5 fr_link4_X_fr_link5;
    Type_fr_link6_X_fr_link5 fr_link6_X_fr_link5;
    Type_fr_link5_X_fr_link6 fr_link5_X_fr_link6;
    Type_fr_link7_X_fr_link6 fr_link7_X_fr_link6;
    Type_fr_link6_X_fr_link7 fr_link6_X_fr_link7;

protected:

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
template <typename TRAIT>
class ForceTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformForce<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_base0_X_fr_link1 : public TransformForce<Scalar, Type_fr_base0_X_fr_link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link1();
        const Type_fr_base0_X_fr_link1& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link2 : public TransformForce<Scalar, Type_fr_base0_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link2();
        const Type_fr_base0_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link3 : public TransformForce<Scalar, Type_fr_base0_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link3();
        const Type_fr_base0_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link4 : public TransformForce<Scalar, Type_fr_base0_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link4();
        const Type_fr_base0_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link5 : public TransformForce<Scalar, Type_fr_base0_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link5();
        const Type_fr_base0_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link6 : public TransformForce<Scalar, Type_fr_base0_X_fr_link6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link6();
        const Type_fr_base0_X_fr_link6& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link7 : public TransformForce<Scalar, Type_fr_base0_X_fr_link7>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link7();
        const Type_fr_base0_X_fr_link7& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_ee : public TransformForce<Scalar, Type_fr_base0_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_ee();
        const Type_fr_base0_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_link1_X_fr_base0 : public TransformForce<Scalar, Type_fr_link1_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link1_X_fr_base0();
        const Type_fr_link1_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_base0 : public TransformForce<Scalar, Type_fr_link2_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_base0();
        const Type_fr_link2_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link3_X_fr_base0 : public TransformForce<Scalar, Type_fr_link3_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_base0();
        const Type_fr_link3_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_base0 : public TransformForce<Scalar, Type_fr_link4_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_base0();
        const Type_fr_link4_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_base0 : public TransformForce<Scalar, Type_fr_link5_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_base0();
        const Type_fr_link5_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link6_X_fr_base0 : public TransformForce<Scalar, Type_fr_link6_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link6_X_fr_base0();
        const Type_fr_link6_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link7_X_fr_base0 : public TransformForce<Scalar, Type_fr_link7_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link7_X_fr_base0();
        const Type_fr_link7_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_X_fr_base0 : public TransformForce<Scalar, Type_fr_ee_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_X_fr_base0();
        const Type_fr_ee_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jA : public TransformForce<Scalar, Type_fr_base0_X_fr_jA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jA();
        const Type_fr_base0_X_fr_jA& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jB : public TransformForce<Scalar, Type_fr_base0_X_fr_jB>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jB();
        const Type_fr_base0_X_fr_jB& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jC : public TransformForce<Scalar, Type_fr_base0_X_fr_jC>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jC();
        const Type_fr_base0_X_fr_jC& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jD : public TransformForce<Scalar, Type_fr_base0_X_fr_jD>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jD();
        const Type_fr_base0_X_fr_jD& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jE : public TransformForce<Scalar, Type_fr_base0_X_fr_jE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jE();
        const Type_fr_base0_X_fr_jE& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jF : public TransformForce<Scalar, Type_fr_base0_X_fr_jF>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jF();
        const Type_fr_base0_X_fr_jF& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jG : public TransformForce<Scalar, Type_fr_base0_X_fr_jG>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jG();
        const Type_fr_base0_X_fr_jG& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_link1 : public TransformForce<Scalar, Type_fr_link2_X_fr_link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_link1();
        const Type_fr_link2_X_fr_link1& update(const JState&);
    protected:
    };
    
    class Type_fr_link1_X_fr_link2 : public TransformForce<Scalar, Type_fr_link1_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link1_X_fr_link2();
        const Type_fr_link1_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_link3_X_fr_link2 : public TransformForce<Scalar, Type_fr_link3_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_link2();
        const Type_fr_link3_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_link3 : public TransformForce<Scalar, Type_fr_link2_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_link3();
        const Type_fr_link2_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_link3 : public TransformForce<Scalar, Type_fr_link4_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_link3();
        const Type_fr_link4_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_link3_X_fr_link4 : public TransformForce<Scalar, Type_fr_link3_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_link4();
        const Type_fr_link3_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_link4 : public TransformForce<Scalar, Type_fr_link5_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_link4();
        const Type_fr_link5_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_link5 : public TransformForce<Scalar, Type_fr_link4_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_link5();
        const Type_fr_link4_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_link6_X_fr_link5 : public TransformForce<Scalar, Type_fr_link6_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link6_X_fr_link5();
        const Type_fr_link6_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_link6 : public TransformForce<Scalar, Type_fr_link5_X_fr_link6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_link6();
        const Type_fr_link5_X_fr_link6& update(const JState&);
    protected:
    };
    
    class Type_fr_link7_X_fr_link6 : public TransformForce<Scalar, Type_fr_link7_X_fr_link6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link7_X_fr_link6();
        const Type_fr_link7_X_fr_link6& update(const JState&);
    protected:
    };
    
    class Type_fr_link6_X_fr_link7 : public TransformForce<Scalar, Type_fr_link6_X_fr_link7>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link6_X_fr_link7();
        const Type_fr_link6_X_fr_link7& update(const JState&);
    protected:
    };
    
public:
    ForceTransforms();
    void updateParameters();
    Type_fr_base0_X_fr_link1 fr_base0_X_fr_link1;
    Type_fr_base0_X_fr_link2 fr_base0_X_fr_link2;
    Type_fr_base0_X_fr_link3 fr_base0_X_fr_link3;
    Type_fr_base0_X_fr_link4 fr_base0_X_fr_link4;
    Type_fr_base0_X_fr_link5 fr_base0_X_fr_link5;
    Type_fr_base0_X_fr_link6 fr_base0_X_fr_link6;
    Type_fr_base0_X_fr_link7 fr_base0_X_fr_link7;
    Type_fr_base0_X_fr_ee fr_base0_X_fr_ee;
    Type_fr_link1_X_fr_base0 fr_link1_X_fr_base0;
    Type_fr_link2_X_fr_base0 fr_link2_X_fr_base0;
    Type_fr_link3_X_fr_base0 fr_link3_X_fr_base0;
    Type_fr_link4_X_fr_base0 fr_link4_X_fr_base0;
    Type_fr_link5_X_fr_base0 fr_link5_X_fr_base0;
    Type_fr_link6_X_fr_base0 fr_link6_X_fr_base0;
    Type_fr_link7_X_fr_base0 fr_link7_X_fr_base0;
    Type_fr_ee_X_fr_base0 fr_ee_X_fr_base0;
    Type_fr_base0_X_fr_jA fr_base0_X_fr_jA;
    Type_fr_base0_X_fr_jB fr_base0_X_fr_jB;
    Type_fr_base0_X_fr_jC fr_base0_X_fr_jC;
    Type_fr_base0_X_fr_jD fr_base0_X_fr_jD;
    Type_fr_base0_X_fr_jE fr_base0_X_fr_jE;
    Type_fr_base0_X_fr_jF fr_base0_X_fr_jF;
    Type_fr_base0_X_fr_jG fr_base0_X_fr_jG;
    Type_fr_link2_X_fr_link1 fr_link2_X_fr_link1;
    Type_fr_link1_X_fr_link2 fr_link1_X_fr_link2;
    Type_fr_link3_X_fr_link2 fr_link3_X_fr_link2;
    Type_fr_link2_X_fr_link3 fr_link2_X_fr_link3;
    Type_fr_link4_X_fr_link3 fr_link4_X_fr_link3;
    Type_fr_link3_X_fr_link4 fr_link3_X_fr_link4;
    Type_fr_link5_X_fr_link4 fr_link5_X_fr_link4;
    Type_fr_link4_X_fr_link5 fr_link4_X_fr_link5;
    Type_fr_link6_X_fr_link5 fr_link6_X_fr_link5;
    Type_fr_link5_X_fr_link6 fr_link5_X_fr_link6;
    Type_fr_link7_X_fr_link6 fr_link7_X_fr_link6;
    Type_fr_link6_X_fr_link7 fr_link6_X_fr_link7;

protected:

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
template <typename TRAIT>
class HomogeneousTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformHomogeneous<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_base0_X_fr_link1 : public TransformHomogeneous<Scalar, Type_fr_base0_X_fr_link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link1();
        const Type_fr_base0_X_fr_link1& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link2 : public TransformHomogeneous<Scalar, Type_fr_base0_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link2();
        const Type_fr_base0_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link3 : public TransformHomogeneous<Scalar, Type_fr_base0_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link3();
        const Type_fr_base0_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link4 : public TransformHomogeneous<Scalar, Type_fr_base0_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link4();
        const Type_fr_base0_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link5 : public TransformHomogeneous<Scalar, Type_fr_base0_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link5();
        const Type_fr_base0_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link6 : public TransformHomogeneous<Scalar, Type_fr_base0_X_fr_link6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link6();
        const Type_fr_base0_X_fr_link6& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_link7 : public TransformHomogeneous<Scalar, Type_fr_base0_X_fr_link7>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_link7();
        const Type_fr_base0_X_fr_link7& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_ee : public TransformHomogeneous<Scalar, Type_fr_base0_X_fr_ee>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_ee();
        const Type_fr_base0_X_fr_ee& update(const JState&);
    protected:
    };
    
    class Type_fr_link1_X_fr_base0 : public TransformHomogeneous<Scalar, Type_fr_link1_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link1_X_fr_base0();
        const Type_fr_link1_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_base0 : public TransformHomogeneous<Scalar, Type_fr_link2_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_base0();
        const Type_fr_link2_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link3_X_fr_base0 : public TransformHomogeneous<Scalar, Type_fr_link3_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_base0();
        const Type_fr_link3_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_base0 : public TransformHomogeneous<Scalar, Type_fr_link4_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_base0();
        const Type_fr_link4_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_base0 : public TransformHomogeneous<Scalar, Type_fr_link5_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_base0();
        const Type_fr_link5_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link6_X_fr_base0 : public TransformHomogeneous<Scalar, Type_fr_link6_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link6_X_fr_base0();
        const Type_fr_link6_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_link7_X_fr_base0 : public TransformHomogeneous<Scalar, Type_fr_link7_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link7_X_fr_base0();
        const Type_fr_link7_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_ee_X_fr_base0 : public TransformHomogeneous<Scalar, Type_fr_ee_X_fr_base0>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_ee_X_fr_base0();
        const Type_fr_ee_X_fr_base0& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jA : public TransformHomogeneous<Scalar, Type_fr_base0_X_fr_jA>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jA();
        const Type_fr_base0_X_fr_jA& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jB : public TransformHomogeneous<Scalar, Type_fr_base0_X_fr_jB>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jB();
        const Type_fr_base0_X_fr_jB& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jC : public TransformHomogeneous<Scalar, Type_fr_base0_X_fr_jC>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jC();
        const Type_fr_base0_X_fr_jC& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jD : public TransformHomogeneous<Scalar, Type_fr_base0_X_fr_jD>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jD();
        const Type_fr_base0_X_fr_jD& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jE : public TransformHomogeneous<Scalar, Type_fr_base0_X_fr_jE>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jE();
        const Type_fr_base0_X_fr_jE& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jF : public TransformHomogeneous<Scalar, Type_fr_base0_X_fr_jF>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jF();
        const Type_fr_base0_X_fr_jF& update(const JState&);
    protected:
    };
    
    class Type_fr_base0_X_fr_jG : public TransformHomogeneous<Scalar, Type_fr_base0_X_fr_jG>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base0_X_fr_jG();
        const Type_fr_base0_X_fr_jG& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_link1 : public TransformHomogeneous<Scalar, Type_fr_link2_X_fr_link1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_link1();
        const Type_fr_link2_X_fr_link1& update(const JState&);
    protected:
    };
    
    class Type_fr_link1_X_fr_link2 : public TransformHomogeneous<Scalar, Type_fr_link1_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link1_X_fr_link2();
        const Type_fr_link1_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_link3_X_fr_link2 : public TransformHomogeneous<Scalar, Type_fr_link3_X_fr_link2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_link2();
        const Type_fr_link3_X_fr_link2& update(const JState&);
    protected:
    };
    
    class Type_fr_link2_X_fr_link3 : public TransformHomogeneous<Scalar, Type_fr_link2_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link2_X_fr_link3();
        const Type_fr_link2_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_link3 : public TransformHomogeneous<Scalar, Type_fr_link4_X_fr_link3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_link3();
        const Type_fr_link4_X_fr_link3& update(const JState&);
    protected:
    };
    
    class Type_fr_link3_X_fr_link4 : public TransformHomogeneous<Scalar, Type_fr_link3_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link3_X_fr_link4();
        const Type_fr_link3_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_link4 : public TransformHomogeneous<Scalar, Type_fr_link5_X_fr_link4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_link4();
        const Type_fr_link5_X_fr_link4& update(const JState&);
    protected:
    };
    
    class Type_fr_link4_X_fr_link5 : public TransformHomogeneous<Scalar, Type_fr_link4_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link4_X_fr_link5();
        const Type_fr_link4_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_link6_X_fr_link5 : public TransformHomogeneous<Scalar, Type_fr_link6_X_fr_link5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link6_X_fr_link5();
        const Type_fr_link6_X_fr_link5& update(const JState&);
    protected:
    };
    
    class Type_fr_link5_X_fr_link6 : public TransformHomogeneous<Scalar, Type_fr_link5_X_fr_link6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link5_X_fr_link6();
        const Type_fr_link5_X_fr_link6& update(const JState&);
    protected:
    };
    
    class Type_fr_link7_X_fr_link6 : public TransformHomogeneous<Scalar, Type_fr_link7_X_fr_link6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link7_X_fr_link6();
        const Type_fr_link7_X_fr_link6& update(const JState&);
    protected:
    };
    
    class Type_fr_link6_X_fr_link7 : public TransformHomogeneous<Scalar, Type_fr_link6_X_fr_link7>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_link6_X_fr_link7();
        const Type_fr_link6_X_fr_link7& update(const JState&);
    protected:
    };
    
public:
    HomogeneousTransforms();
    void updateParameters();
    Type_fr_base0_X_fr_link1 fr_base0_X_fr_link1;
    Type_fr_base0_X_fr_link2 fr_base0_X_fr_link2;
    Type_fr_base0_X_fr_link3 fr_base0_X_fr_link3;
    Type_fr_base0_X_fr_link4 fr_base0_X_fr_link4;
    Type_fr_base0_X_fr_link5 fr_base0_X_fr_link5;
    Type_fr_base0_X_fr_link6 fr_base0_X_fr_link6;
    Type_fr_base0_X_fr_link7 fr_base0_X_fr_link7;
    Type_fr_base0_X_fr_ee fr_base0_X_fr_ee;
    Type_fr_link1_X_fr_base0 fr_link1_X_fr_base0;
    Type_fr_link2_X_fr_base0 fr_link2_X_fr_base0;
    Type_fr_link3_X_fr_base0 fr_link3_X_fr_base0;
    Type_fr_link4_X_fr_base0 fr_link4_X_fr_base0;
    Type_fr_link5_X_fr_base0 fr_link5_X_fr_base0;
    Type_fr_link6_X_fr_base0 fr_link6_X_fr_base0;
    Type_fr_link7_X_fr_base0 fr_link7_X_fr_base0;
    Type_fr_ee_X_fr_base0 fr_ee_X_fr_base0;
    Type_fr_base0_X_fr_jA fr_base0_X_fr_jA;
    Type_fr_base0_X_fr_jB fr_base0_X_fr_jB;
    Type_fr_base0_X_fr_jC fr_base0_X_fr_jC;
    Type_fr_base0_X_fr_jD fr_base0_X_fr_jD;
    Type_fr_base0_X_fr_jE fr_base0_X_fr_jE;
    Type_fr_base0_X_fr_jF fr_base0_X_fr_jF;
    Type_fr_base0_X_fr_jG fr_base0_X_fr_jG;
    Type_fr_link2_X_fr_link1 fr_link2_X_fr_link1;
    Type_fr_link1_X_fr_link2 fr_link1_X_fr_link2;
    Type_fr_link3_X_fr_link2 fr_link3_X_fr_link2;
    Type_fr_link2_X_fr_link3 fr_link2_X_fr_link3;
    Type_fr_link4_X_fr_link3 fr_link4_X_fr_link3;
    Type_fr_link3_X_fr_link4 fr_link3_X_fr_link4;
    Type_fr_link5_X_fr_link4 fr_link5_X_fr_link4;
    Type_fr_link4_X_fr_link5 fr_link4_X_fr_link5;
    Type_fr_link6_X_fr_link5 fr_link6_X_fr_link5;
    Type_fr_link5_X_fr_link6 fr_link5_X_fr_link6;
    Type_fr_link7_X_fr_link6 fr_link7_X_fr_link6;
    Type_fr_link6_X_fr_link7 fr_link6_X_fr_link7;

protected:

}; //class 'HomogeneousTransforms'

}

using MotionTransforms = tpl::MotionTransforms<rbd::DoubleTrait>;
using ForceTransforms = tpl::ForceTransforms<rbd::DoubleTrait>;
using HomogeneousTransforms = tpl::HomogeneousTransforms<rbd::DoubleTrait>;

}
}

#include "transforms.impl.h"

#endif
