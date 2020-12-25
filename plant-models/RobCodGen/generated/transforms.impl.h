
// Constructors
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::MotionTransforms
    ()
     :
    fr_base0_X_fr_link1(),
    fr_base0_X_fr_link2(),
    fr_base0_X_fr_link3(),
    fr_base0_X_fr_link4(),
    fr_base0_X_fr_link5(),
    fr_base0_X_fr_link6(),
    fr_base0_X_fr_link7(),
    fr_base0_X_fr_ee(),
    fr_link1_X_fr_base0(),
    fr_link2_X_fr_base0(),
    fr_link3_X_fr_base0(),
    fr_link4_X_fr_base0(),
    fr_link5_X_fr_base0(),
    fr_link6_X_fr_base0(),
    fr_link7_X_fr_base0(),
    fr_ee_X_fr_base0(),
    fr_base0_X_fr_jA(),
    fr_base0_X_fr_jB(),
    fr_base0_X_fr_jC(),
    fr_base0_X_fr_jD(),
    fr_base0_X_fr_jE(),
    fr_base0_X_fr_jF(),
    fr_base0_X_fr_jG(),
    fr_link2_X_fr_link1(),
    fr_link1_X_fr_link2(),
    fr_link3_X_fr_link2(),
    fr_link2_X_fr_link3(),
    fr_link4_X_fr_link3(),
    fr_link3_X_fr_link4(),
    fr_link5_X_fr_link4(),
    fr_link4_X_fr_link5(),
    fr_link6_X_fr_link5(),
    fr_link5_X_fr_link6(),
    fr_link7_X_fr_link6(),
    fr_link6_X_fr_link7()
{
    updateParameters();
}
template <typename TRAIT>
void iit::Kuka::tpl::MotionTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::ForceTransforms
    ()
     :
    fr_base0_X_fr_link1(),
    fr_base0_X_fr_link2(),
    fr_base0_X_fr_link3(),
    fr_base0_X_fr_link4(),
    fr_base0_X_fr_link5(),
    fr_base0_X_fr_link6(),
    fr_base0_X_fr_link7(),
    fr_base0_X_fr_ee(),
    fr_link1_X_fr_base0(),
    fr_link2_X_fr_base0(),
    fr_link3_X_fr_base0(),
    fr_link4_X_fr_base0(),
    fr_link5_X_fr_base0(),
    fr_link6_X_fr_base0(),
    fr_link7_X_fr_base0(),
    fr_ee_X_fr_base0(),
    fr_base0_X_fr_jA(),
    fr_base0_X_fr_jB(),
    fr_base0_X_fr_jC(),
    fr_base0_X_fr_jD(),
    fr_base0_X_fr_jE(),
    fr_base0_X_fr_jF(),
    fr_base0_X_fr_jG(),
    fr_link2_X_fr_link1(),
    fr_link1_X_fr_link2(),
    fr_link3_X_fr_link2(),
    fr_link2_X_fr_link3(),
    fr_link4_X_fr_link3(),
    fr_link3_X_fr_link4(),
    fr_link5_X_fr_link4(),
    fr_link4_X_fr_link5(),
    fr_link6_X_fr_link5(),
    fr_link5_X_fr_link6(),
    fr_link7_X_fr_link6(),
    fr_link6_X_fr_link7()
{
    updateParameters();
}
template <typename TRAIT>
void iit::Kuka::tpl::ForceTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::HomogeneousTransforms
    ()
     :
    fr_base0_X_fr_link1(),
    fr_base0_X_fr_link2(),
    fr_base0_X_fr_link3(),
    fr_base0_X_fr_link4(),
    fr_base0_X_fr_link5(),
    fr_base0_X_fr_link6(),
    fr_base0_X_fr_link7(),
    fr_base0_X_fr_ee(),
    fr_link1_X_fr_base0(),
    fr_link2_X_fr_base0(),
    fr_link3_X_fr_base0(),
    fr_link4_X_fr_base0(),
    fr_link5_X_fr_base0(),
    fr_link6_X_fr_base0(),
    fr_link7_X_fr_base0(),
    fr_ee_X_fr_base0(),
    fr_base0_X_fr_jA(),
    fr_base0_X_fr_jB(),
    fr_base0_X_fr_jC(),
    fr_base0_X_fr_jD(),
    fr_base0_X_fr_jE(),
    fr_base0_X_fr_jF(),
    fr_base0_X_fr_jG(),
    fr_link2_X_fr_link1(),
    fr_link1_X_fr_link2(),
    fr_link3_X_fr_link2(),
    fr_link2_X_fr_link3(),
    fr_link4_X_fr_link3(),
    fr_link3_X_fr_link4(),
    fr_link5_X_fr_link4(),
    fr_link4_X_fr_link5(),
    fr_link6_X_fr_link5(),
    fr_link5_X_fr_link6(),
    fr_link7_X_fr_link6(),
    fr_link6_X_fr_link7()
{
    updateParameters();
}
template <typename TRAIT>
void iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link1::Type_fr_base0_X_fr_link1()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link1& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link1::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar c_q_jA_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    c_q_jA_ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  c_q_jA_;
    (*this)(0,1) = - s_q_jA_;
    (*this)(1,0) =  s_q_jA_;
    (*this)(1,1) =  c_q_jA_;
    (*this)(3,0) = (- 0.1575 *  s_q_jA_);
    (*this)(3,1) = (- 0.1575 *  c_q_jA_);
    (*this)(3,3) =  c_q_jA_;
    (*this)(3,4) = - s_q_jA_;
    (*this)(4,0) = ( 0.1575 *  c_q_jA_);
    (*this)(4,1) = (- 0.1575 *  s_q_jA_);
    (*this)(4,3) =  s_q_jA_;
    (*this)(4,4) =  c_q_jA_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link2::Type_fr_base0_X_fr_link2()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link2& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link2::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar s_q_jA_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jA_ = TRAIT::sin( q(JA));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = (- c_q_jA_ *  c_q_jB_);
    (*this)(0,1) = ( c_q_jA_ *  s_q_jB_);
    (*this)(0,2) = - s_q_jA_;
    (*this)(1,0) = (- s_q_jA_ *  c_q_jB_);
    (*this)(1,1) = ( s_q_jA_ *  s_q_jB_);
    (*this)(1,2) =  c_q_jA_;
    (*this)(2,0) =  s_q_jB_;
    (*this)(2,1) =  c_q_jB_;
    (*this)(3,0) = (( 0.36 *  s_q_jA_) *  c_q_jB_);
    (*this)(3,1) = ((- 0.36 *  s_q_jA_) *  s_q_jB_);
    (*this)(3,2) = (- 0.36 *  c_q_jA_);
    (*this)(3,3) = (- c_q_jA_ *  c_q_jB_);
    (*this)(3,4) = ( c_q_jA_ *  s_q_jB_);
    (*this)(3,5) = - s_q_jA_;
    (*this)(4,0) = ((- 0.36 *  c_q_jA_) *  c_q_jB_);
    (*this)(4,1) = (( 0.36 *  c_q_jA_) *  s_q_jB_);
    (*this)(4,2) = (- 0.36 *  s_q_jA_);
    (*this)(4,3) = (- s_q_jA_ *  c_q_jB_);
    (*this)(4,4) = ( s_q_jA_ *  s_q_jB_);
    (*this)(4,5) =  c_q_jA_;
    (*this)(5,3) =  s_q_jB_;
    (*this)(5,4) =  c_q_jB_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link3::Type_fr_base0_X_fr_link3()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link3& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link3::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jB_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    
    (*this)(0,0) = ((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_));
    (*this)(0,1) = (((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_));
    (*this)(0,2) = ( c_q_jA_ *  s_q_jB_);
    (*this)(1,0) = (( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_));
    (*this)(1,1) = (( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_));
    (*this)(1,2) = ( s_q_jA_ *  s_q_jB_);
    (*this)(2,0) = (- s_q_jB_ *  c_q_jC_);
    (*this)(2,1) = ( s_q_jB_ *  s_q_jC_);
    (*this)(2,2) =  c_q_jB_;
    (*this)(3,0) = ((((- 0.36 *  c_q_jA_) - (( 0.2045 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.2045 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(3,1) = (((( 0.2045 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.36 *  c_q_jA_) - (( 0.2045 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(3,2) = ((- 0.36 *  s_q_jA_) *  s_q_jB_);
    (*this)(3,3) = ((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_));
    (*this)(3,4) = (((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_));
    (*this)(3,5) = ( c_q_jA_ *  s_q_jB_);
    (*this)(4,0) = ((((- 0.36 *  s_q_jA_) - (( 0.2045 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.2045 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(4,1) = ((((- 0.2045 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.36 *  s_q_jA_) - (( 0.2045 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(4,2) = (( 0.36 *  c_q_jA_) *  s_q_jB_);
    (*this)(4,3) = (( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_));
    (*this)(4,4) = (( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_));
    (*this)(4,5) = ( s_q_jA_ *  s_q_jB_);
    (*this)(5,0) = (( 0.2045 *  s_q_jB_) *  s_q_jC_);
    (*this)(5,1) = (( 0.2045 *  s_q_jB_) *  c_q_jC_);
    (*this)(5,3) = (- s_q_jB_ *  c_q_jC_);
    (*this)(5,4) = ( s_q_jB_ *  s_q_jC_);
    (*this)(5,5) =  c_q_jB_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link4::Type_fr_base0_X_fr_link4()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link4& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link4::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jD_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jD_ = TRAIT::cos( q(JD));
    
    (*this)(0,0) = ((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(0,1) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(0,2) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(1,0) = ((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(1,1) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,2) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(2,0) = (( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_));
    (*this)(2,1) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(2,2) = (- s_q_jB_ *  s_q_jC_);
    (*this)(3,0) = (((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_));
    (*this)(3,1) = ((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(3,2) = ((((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(3,3) = ((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(3,4) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(3,5) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(4,0) = (((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_));
    (*this)(4,1) = (((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(4,2) = (((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(4,3) = ((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(4,4) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(4,5) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(5,0) = ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_);
    (*this)(5,1) = (((- 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_);
    (*this)(5,2) = ((- 0.42 *  s_q_jB_) *  c_q_jC_);
    (*this)(5,3) = (( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_));
    (*this)(5,4) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(5,5) = (- s_q_jB_ *  s_q_jC_);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link5::Type_fr_base0_X_fr_link5()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link5& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link5::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jE_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar c_q_jC_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jD_;
    Scalar c_q_jE_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jE_ = TRAIT::cos( q(JE));
    
    (*this)(0,0) = (((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,1) = ((((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(0,2) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,0) = ((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(1,1) = (((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  c_q_jE_));
    (*this)(1,2) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(2,0) = ((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_));
    (*this)(2,1) = (((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(2,2) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(3,0) = ((((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.1845 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.1845 *  s_q_jA_) *  c_q_jC_) + ((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,1) = (((((((- 0.1845 *  s_q_jA_) *  c_q_jC_) - ((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.1845 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,2) = ((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(3,3) = (((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,4) = ((((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(3,5) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(4,0) = ((((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.1845 *  c_q_jA_) *  s_q_jC_) + ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.1845 *  c_q_jA_) *  c_q_jC_) + ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(4,1) = ((((((( 0.1845 *  c_q_jA_) *  c_q_jC_) - ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.1845 *  c_q_jA_) *  s_q_jC_) + ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(4,2) = (((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(4,3) = ((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(4,4) = (((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  c_q_jE_));
    (*this)(4,5) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(5,0) = ((((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.1845 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.1845 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((- 0.1845 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_));
    (*this)(5,1) = ((((( 0.1845 *  s_q_jB_) *  s_q_jC_) + ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  s_q_jE_) + (((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.1845 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.1845 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(5,2) = (((- 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_);
    (*this)(5,3) = ((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_));
    (*this)(5,4) = (((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(5,5) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link6::Type_fr_base0_X_fr_link6()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link6& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link6::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jD_;
    Scalar s_q_jF_;
    Scalar s_q_jE_;
    Scalar c_q_jA_;
    Scalar c_q_jD_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jF_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jF_ = TRAIT::sin( q(JF));
    s_q_jE_ = TRAIT::sin( q(JE));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jF_ = TRAIT::cos( q(JF));
    
    (*this)(0,0) = ((((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,1) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(0,2) = (((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(1,0) = ((((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(1,1) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,2) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(2,0) = (((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + (((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(2,1) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,2) = ((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(3,0) = (((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + ((((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + ((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(3,1) = ((((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  c_q_jF_));
    (*this)(3,2) = ((((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,3) = ((((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(3,4) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(3,5) = ((((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(4,0) = (((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + (((((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + ((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(4,1) = (((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,2) = (((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(4,3) = ((((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(4,4) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,5) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(5,0) = ((((((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((( 0.4 *  c_q_jB_) *  s_q_jD_) - ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_));
    (*this)(5,1) = (((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  c_q_jF_));
    (*this)(5,2) = (((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  s_q_jE_) + ((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(5,3) = (((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + (((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(5,4) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,5) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link7::Type_fr_base0_X_fr_link7()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link7& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_link7::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jE_;
    Scalar s_q_jG_;
    Scalar s_q_jF_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jD_;
    Scalar c_q_jF_;
    Scalar c_q_jG_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jG_ = TRAIT::sin( q(JG));
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jF_ = TRAIT::cos( q(JF));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,1) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(0,2) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,0) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,1) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,2) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,0) = ((((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((( s_q_jB_ *  s_q_jC_) *  s_q_jE_) + ((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(2,1) = ((((((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + (((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(2,2) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(3,0) = ((((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.081 *  s_q_jA_) *  c_q_jC_) + ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  s_q_jA_) *  c_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,1) = (((((((((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) + (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.081 *  s_q_jA_) *  c_q_jC_) + ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,2) = ((((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  c_q_jF_));
    (*this)(3,3) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,4) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(3,5) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,0) = ((((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.081 *  c_q_jA_) *  s_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((( 0.081 *  c_q_jA_) *  c_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((((- 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  c_q_jA_) *  c_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,1) = (((((((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  c_q_jA_) *  s_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.081 *  c_q_jA_) *  s_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,2) = (((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,3) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,4) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(4,5) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,0) = ((((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.081 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.081 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((( 0.081 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 0.4 *  s_q_jB_) *  s_q_jC_) + ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  c_q_jG_));
    (*this)(5,1) = ((((((((- 0.081 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + (((( 0.081 *  c_q_jB_) *  s_q_jD_) - ((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_)) + (((((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.081 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.081 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(5,2) = (((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  c_q_jF_));
    (*this)(5,3) = ((((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((( s_q_jB_ *  s_q_jC_) *  s_q_jE_) + ((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(5,4) = ((((((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + (((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(5,5) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_ee::Type_fr_base0_X_fr_ee()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_ee& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_ee::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jE_;
    Scalar s_q_jG_;
    Scalar s_q_jF_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jD_;
    Scalar c_q_jF_;
    Scalar c_q_jG_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jG_ = TRAIT::sin( q(JG));
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jF_ = TRAIT::cos( q(JF));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,1) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(0,2) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,0) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,1) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,2) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,0) = ((((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((( s_q_jB_ *  s_q_jC_) *  s_q_jE_) + ((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(2,1) = ((((((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + (((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(2,2) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(3,0) = ((((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.241 *  s_q_jA_) *  c_q_jC_) + ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  s_q_jA_) *  c_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,1) = (((((((((( 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) + (( 0.241 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.241 *  s_q_jA_) *  c_q_jC_) + ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,2) = ((((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  c_q_jF_));
    (*this)(3,3) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,4) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(3,5) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,0) = ((((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.241 *  c_q_jA_) *  s_q_jC_) - ((( 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((( 0.241 *  c_q_jA_) *  c_q_jC_) - ((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((((- 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_) - ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  c_q_jA_) *  c_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,1) = (((((((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  c_q_jA_) *  s_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.241 *  c_q_jA_) *  s_q_jC_) - ((( 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,2) = (((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,3) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,4) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(4,5) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,0) = ((((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.241 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.241 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.241 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((( 0.241 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.241 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 0.4 *  s_q_jB_) *  s_q_jC_) + ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  c_q_jG_));
    (*this)(5,1) = ((((((((- 0.241 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + (((( 0.241 *  c_q_jB_) *  s_q_jD_) - ((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_)) + (((((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.241 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.241 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.241 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(5,2) = (((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  c_q_jF_));
    (*this)(5,3) = ((((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((( s_q_jB_ *  s_q_jC_) *  s_q_jE_) + ((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(5,4) = ((((((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + (((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(5,5) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link1_X_fr_base0::Type_fr_link1_X_fr_base0()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link1_X_fr_base0& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link1_X_fr_base0::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar c_q_jA_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    c_q_jA_ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  c_q_jA_;
    (*this)(0,1) =  s_q_jA_;
    (*this)(1,0) = - s_q_jA_;
    (*this)(1,1) =  c_q_jA_;
    (*this)(3,0) = (- 0.1575 *  s_q_jA_);
    (*this)(3,1) = ( 0.1575 *  c_q_jA_);
    (*this)(3,3) =  c_q_jA_;
    (*this)(3,4) =  s_q_jA_;
    (*this)(4,0) = (- 0.1575 *  c_q_jA_);
    (*this)(4,1) = (- 0.1575 *  s_q_jA_);
    (*this)(4,3) = - s_q_jA_;
    (*this)(4,4) =  c_q_jA_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link2_X_fr_base0::Type_fr_link2_X_fr_base0()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(4,2) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link2_X_fr_base0& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link2_X_fr_base0::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = (- c_q_jA_ *  c_q_jB_);
    (*this)(0,1) = (- s_q_jA_ *  c_q_jB_);
    (*this)(0,2) =  s_q_jB_;
    (*this)(1,0) = ( c_q_jA_ *  s_q_jB_);
    (*this)(1,1) = ( s_q_jA_ *  s_q_jB_);
    (*this)(1,2) =  c_q_jB_;
    (*this)(2,0) = - s_q_jA_;
    (*this)(2,1) =  c_q_jA_;
    (*this)(3,0) = (( 0.36 *  s_q_jA_) *  c_q_jB_);
    (*this)(3,1) = ((- 0.36 *  c_q_jA_) *  c_q_jB_);
    (*this)(3,3) = (- c_q_jA_ *  c_q_jB_);
    (*this)(3,4) = (- s_q_jA_ *  c_q_jB_);
    (*this)(3,5) =  s_q_jB_;
    (*this)(4,0) = ((- 0.36 *  s_q_jA_) *  s_q_jB_);
    (*this)(4,1) = (( 0.36 *  c_q_jA_) *  s_q_jB_);
    (*this)(4,3) = ( c_q_jA_ *  s_q_jB_);
    (*this)(4,4) = ( s_q_jA_ *  s_q_jB_);
    (*this)(4,5) =  c_q_jB_;
    (*this)(5,0) = (- 0.36 *  c_q_jA_);
    (*this)(5,1) = (- 0.36 *  s_q_jA_);
    (*this)(5,3) = - s_q_jA_;
    (*this)(5,4) =  c_q_jA_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link3_X_fr_base0::Type_fr_link3_X_fr_base0()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link3_X_fr_base0& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link3_X_fr_base0::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jB_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    
    (*this)(0,0) = ((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_));
    (*this)(0,1) = (( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_));
    (*this)(0,2) = (- s_q_jB_ *  c_q_jC_);
    (*this)(1,0) = (((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_));
    (*this)(1,1) = (( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_));
    (*this)(1,2) = ( s_q_jB_ *  s_q_jC_);
    (*this)(2,0) = ( c_q_jA_ *  s_q_jB_);
    (*this)(2,1) = ( s_q_jA_ *  s_q_jB_);
    (*this)(2,2) =  c_q_jB_;
    (*this)(3,0) = ((((- 0.36 *  c_q_jA_) - (( 0.2045 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.2045 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(3,1) = ((((- 0.36 *  s_q_jA_) - (( 0.2045 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.2045 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(3,2) = (( 0.2045 *  s_q_jB_) *  s_q_jC_);
    (*this)(3,3) = ((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_));
    (*this)(3,4) = (( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_));
    (*this)(3,5) = (- s_q_jB_ *  c_q_jC_);
    (*this)(4,0) = (((( 0.2045 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.36 *  c_q_jA_) - (( 0.2045 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(4,1) = ((((- 0.2045 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.36 *  s_q_jA_) - (( 0.2045 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(4,2) = (( 0.2045 *  s_q_jB_) *  c_q_jC_);
    (*this)(4,3) = (((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_));
    (*this)(4,4) = (( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_));
    (*this)(4,5) = ( s_q_jB_ *  s_q_jC_);
    (*this)(5,0) = ((- 0.36 *  s_q_jA_) *  s_q_jB_);
    (*this)(5,1) = (( 0.36 *  c_q_jA_) *  s_q_jB_);
    (*this)(5,3) = ( c_q_jA_ *  s_q_jB_);
    (*this)(5,4) = ( s_q_jA_ *  s_q_jB_);
    (*this)(5,5) =  c_q_jB_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link4_X_fr_base0::Type_fr_link4_X_fr_base0()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link4_X_fr_base0& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link4_X_fr_base0::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jD_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jD_ = TRAIT::cos( q(JD));
    
    (*this)(0,0) = ((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(0,1) = ((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(0,2) = (( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_));
    (*this)(1,0) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,1) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,2) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(2,0) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(2,1) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(2,2) = (- s_q_jB_ *  s_q_jC_);
    (*this)(3,0) = (((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_));
    (*this)(3,1) = (((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_));
    (*this)(3,2) = ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_);
    (*this)(3,3) = ((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(3,4) = ((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(3,5) = (( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_));
    (*this)(4,0) = ((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(4,1) = (((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(4,2) = (((- 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_);
    (*this)(4,3) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(4,4) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(4,5) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(5,0) = ((((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(5,1) = (((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(5,2) = ((- 0.42 *  s_q_jB_) *  c_q_jC_);
    (*this)(5,3) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(5,4) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(5,5) = (- s_q_jB_ *  s_q_jC_);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link5_X_fr_base0::Type_fr_link5_X_fr_base0()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link5_X_fr_base0& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link5_X_fr_base0::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jE_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar c_q_jC_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jD_;
    Scalar c_q_jE_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jE_ = TRAIT::cos( q(JE));
    
    (*this)(0,0) = (((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,1) = ((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,2) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_));
    (*this)(1,0) = ((((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(1,1) = (((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  c_q_jE_));
    (*this)(1,2) = ((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(2,0) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(2,1) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(2,2) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(3,0) = ((((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.1845 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.1845 *  s_q_jA_) *  c_q_jC_) + ((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,1) = ((((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.1845 *  c_q_jA_) *  s_q_jC_) + ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.1845 *  c_q_jA_) *  c_q_jC_) + ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,2) = ((((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.1845 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.1845 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((- 0.1845 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_));
    (*this)(3,3) = (((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,4) = ((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,5) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_));
    (*this)(4,0) = (((((((- 0.1845 *  s_q_jA_) *  c_q_jC_) - ((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.1845 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(4,1) = ((((((( 0.1845 *  c_q_jA_) *  c_q_jC_) - ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.1845 *  c_q_jA_) *  s_q_jC_) + ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(4,2) = ((((( 0.1845 *  s_q_jB_) *  s_q_jC_) + ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  s_q_jE_) + (((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.1845 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.1845 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(4,3) = ((((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(4,4) = (((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  c_q_jE_));
    (*this)(4,5) = ((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(5,0) = ((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(5,1) = (((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(5,2) = (((- 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_);
    (*this)(5,3) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(5,4) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(5,5) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link6_X_fr_base0::Type_fr_link6_X_fr_base0()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link6_X_fr_base0& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link6_X_fr_base0::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jD_;
    Scalar s_q_jF_;
    Scalar s_q_jE_;
    Scalar c_q_jA_;
    Scalar c_q_jD_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jF_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jF_ = TRAIT::sin( q(JF));
    s_q_jE_ = TRAIT::sin( q(JE));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jF_ = TRAIT::cos( q(JF));
    
    (*this)(0,0) = ((((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,1) = ((((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,2) = ((((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + ((((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(1,0) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,1) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,2) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,0) = (((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(2,1) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(2,2) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(3,0) = (((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + ((((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + ((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(3,1) = (((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + (((((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + ((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(3,2) = ((((((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((( 0.4 *  c_q_jB_) *  s_q_jD_) - ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_));
    (*this)(3,3) = ((((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(3,4) = ((((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(3,5) = ((((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + ((((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(4,0) = ((((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  c_q_jF_));
    (*this)(4,1) = (((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,2) = (((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  c_q_jF_));
    (*this)(4,3) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,4) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,5) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,0) = ((((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(5,1) = (((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(5,2) = (((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  s_q_jE_) + ((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(5,3) = (((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(5,4) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(5,5) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link7_X_fr_base0::Type_fr_link7_X_fr_base0()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link7_X_fr_base0& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link7_X_fr_base0::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jE_;
    Scalar s_q_jG_;
    Scalar s_q_jF_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jD_;
    Scalar c_q_jF_;
    Scalar c_q_jG_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jG_ = TRAIT::sin( q(JG));
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jF_ = TRAIT::cos( q(JF));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,1) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,2) = (((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_) + (((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,0) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,1) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,2) = (((((((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(2,0) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,1) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,2) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(3,0) = ((((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.081 *  s_q_jA_) *  c_q_jC_) + ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  s_q_jA_) *  c_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,1) = ((((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.081 *  c_q_jA_) *  s_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((( 0.081 *  c_q_jA_) *  c_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((((- 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  c_q_jA_) *  c_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,2) = ((((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.081 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.081 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((( 0.081 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 0.4 *  s_q_jB_) *  s_q_jC_) + ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,3) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,4) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,5) = (((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_) + (((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,0) = (((((((((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) + (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.081 *  s_q_jA_) *  c_q_jC_) + ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,1) = (((((((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  c_q_jA_) *  s_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.081 *  c_q_jA_) *  s_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,2) = ((((((((- 0.081 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + (((( 0.081 *  c_q_jB_) *  s_q_jD_) - ((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_)) + (((((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.081 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.081 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,3) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(4,4) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(4,5) = (((((((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(5,0) = ((((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  c_q_jF_));
    (*this)(5,1) = (((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,2) = (((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  c_q_jF_));
    (*this)(5,3) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,4) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,5) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_ee_X_fr_base0::Type_fr_ee_X_fr_base0()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_ee_X_fr_base0& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_ee_X_fr_base0::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jE_;
    Scalar s_q_jG_;
    Scalar s_q_jF_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jD_;
    Scalar c_q_jF_;
    Scalar c_q_jG_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jG_ = TRAIT::sin( q(JG));
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jF_ = TRAIT::cos( q(JF));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,1) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,2) = (((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_) + (((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,0) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,1) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,2) = (((((((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(2,0) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,1) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,2) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(3,0) = ((((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.241 *  s_q_jA_) *  c_q_jC_) + ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  s_q_jA_) *  c_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,1) = ((((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.241 *  c_q_jA_) *  s_q_jC_) - ((( 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((( 0.241 *  c_q_jA_) *  c_q_jC_) - ((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((((- 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_) - ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  c_q_jA_) *  c_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,2) = ((((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.241 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.241 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.241 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((( 0.241 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.241 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 0.4 *  s_q_jB_) *  s_q_jC_) + ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,3) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,4) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,5) = (((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_) + (((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,0) = (((((((((( 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) + (( 0.241 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.241 *  s_q_jA_) *  c_q_jC_) + ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,1) = (((((((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  c_q_jA_) *  s_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.241 *  c_q_jA_) *  s_q_jC_) - ((( 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,2) = ((((((((- 0.241 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + (((( 0.241 *  c_q_jB_) *  s_q_jD_) - ((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_)) + (((((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.241 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.241 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.241 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,3) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(4,4) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(4,5) = (((((((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(5,0) = ((((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  c_q_jF_));
    (*this)(5,1) = (((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,2) = (((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  c_q_jF_));
    (*this)(5,3) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,4) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,5) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jA::Type_fr_base0_X_fr_jA()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = - 0.1575;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0.1575;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jA& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jB::Type_fr_base0_X_fr_jB()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,1) = 0;
    (*this)(3,4) = 0;
    (*this)(4,1) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jB& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jB::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar c_q_jA_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    c_q_jA_ = TRAIT::cos( q(JA));
    
    (*this)(0,0) = - c_q_jA_;
    (*this)(0,2) = - s_q_jA_;
    (*this)(1,0) = - s_q_jA_;
    (*this)(1,2) =  c_q_jA_;
    (*this)(3,0) = ( 0.36 *  s_q_jA_);
    (*this)(3,2) = (- 0.36 *  c_q_jA_);
    (*this)(3,3) = - c_q_jA_;
    (*this)(3,5) = - s_q_jA_;
    (*this)(4,0) = (- 0.36 *  c_q_jA_);
    (*this)(4,2) = (- 0.36 *  s_q_jA_);
    (*this)(4,3) = - s_q_jA_;
    (*this)(4,5) =  c_q_jA_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jC::Type_fr_base0_X_fr_jC()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,1) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,2) = 0;
    (*this)(5,4) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jC& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jC::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = ( c_q_jA_ *  c_q_jB_);
    (*this)(0,1) = - s_q_jA_;
    (*this)(0,2) = ( c_q_jA_ *  s_q_jB_);
    (*this)(1,0) = ( s_q_jA_ *  c_q_jB_);
    (*this)(1,1) =  c_q_jA_;
    (*this)(1,2) = ( s_q_jA_ *  s_q_jB_);
    (*this)(2,0) = - s_q_jB_;
    (*this)(2,2) =  c_q_jB_;
    (*this)(3,0) = (((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.2045 *  s_q_jA_));
    (*this)(3,1) = (((- 0.2045 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_));
    (*this)(3,2) = ((- 0.36 *  s_q_jA_) *  s_q_jB_);
    (*this)(3,3) = ( c_q_jA_ *  c_q_jB_);
    (*this)(3,4) = - s_q_jA_;
    (*this)(3,5) = ( c_q_jA_ *  s_q_jB_);
    (*this)(4,0) = ((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.2045 *  c_q_jA_));
    (*this)(4,1) = (((- 0.2045 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_));
    (*this)(4,2) = (( 0.36 *  c_q_jA_) *  s_q_jB_);
    (*this)(4,3) = ( s_q_jA_ *  c_q_jB_);
    (*this)(4,4) =  c_q_jA_;
    (*this)(4,5) = ( s_q_jA_ *  s_q_jB_);
    (*this)(5,1) = ( 0.2045 *  s_q_jB_);
    (*this)(5,3) = - s_q_jB_;
    (*this)(5,5) =  c_q_jB_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jD::Type_fr_base0_X_fr_jD()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,1) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jD& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jD::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jB_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    
    (*this)(0,0) = ((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_));
    (*this)(0,1) = ( c_q_jA_ *  s_q_jB_);
    (*this)(0,2) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(1,0) = (( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_));
    (*this)(1,1) = ( s_q_jA_ *  s_q_jB_);
    (*this)(1,2) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(2,0) = (- s_q_jB_ *  c_q_jC_);
    (*this)(2,1) =  c_q_jB_;
    (*this)(2,2) = (- s_q_jB_ *  s_q_jC_);
    (*this)(3,0) = ((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(3,1) = ((- 0.36 *  s_q_jA_) *  s_q_jB_);
    (*this)(3,2) = ((((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(3,3) = ((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_));
    (*this)(3,4) = ( c_q_jA_ *  s_q_jB_);
    (*this)(3,5) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(4,0) = ((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(4,1) = (( 0.36 *  c_q_jA_) *  s_q_jB_);
    (*this)(4,2) = (((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(4,3) = (( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_));
    (*this)(4,4) = ( s_q_jA_ *  s_q_jB_);
    (*this)(4,5) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(5,0) = (( 0.42 *  s_q_jB_) *  s_q_jC_);
    (*this)(5,2) = ((- 0.42 *  s_q_jB_) *  c_q_jC_);
    (*this)(5,3) = (- s_q_jB_ *  c_q_jC_);
    (*this)(5,4) =  c_q_jB_;
    (*this)(5,5) = (- s_q_jB_ *  s_q_jC_);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jE::Type_fr_base0_X_fr_jE()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jE& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jE::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jD_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jD_ = TRAIT::cos( q(JD));
    
    (*this)(0,0) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_));
    (*this)(0,1) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(0,2) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,0) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_));
    (*this)(1,1) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(1,2) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(2,0) = ((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_));
    (*this)(2,1) = (- s_q_jB_ *  s_q_jC_);
    (*this)(2,2) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(3,0) = (((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.1845 *  s_q_jA_) *  c_q_jC_));
    (*this)(3,1) = (((((( 0.1845 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.1845 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + ((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(3,2) = ((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(3,3) = ((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_));
    (*this)(3,4) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(3,5) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(4,0) = ((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.1845 *  c_q_jA_) *  c_q_jC_));
    (*this)(4,1) = (((((( 0.1845 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.1845 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + ((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(4,2) = (((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(4,3) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_));
    (*this)(4,4) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(4,5) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(5,0) = ((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.1845 *  s_q_jB_) *  s_q_jC_));
    (*this)(5,1) = (((( 0.1845 *  c_q_jB_) *  s_q_jD_) - ((( 0.1845 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.42 *  s_q_jB_) *  c_q_jC_));
    (*this)(5,2) = (((- 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_);
    (*this)(5,3) = ((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_));
    (*this)(5,4) = (- s_q_jB_ *  s_q_jC_);
    (*this)(5,5) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jF::Type_fr_base0_X_fr_jF()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jF& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jF::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jE_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar c_q_jC_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jD_;
    Scalar c_q_jE_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jE_ = TRAIT::cos( q(JE));
    
    (*this)(0,0) = (((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,1) = ((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(0,2) = (((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(1,0) = ((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(1,1) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,2) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(2,0) = ((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_));
    (*this)(2,1) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(2,2) = ((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(3,0) = ((((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,1) = ((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(3,2) = ((((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,3) = ((((( 1.0 *  s_q_jA_) *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,4) = ((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(3,5) = ((((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(4,0) = ((((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  c_q_jA_) *  s_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(4,1) = (((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(4,2) = (((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(4,3) = (((((( 1.0 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(4,4) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(4,5) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(5,0) = ((((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_));
    (*this)(5,1) = (((- 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_);
    (*this)(5,2) = (((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  s_q_jE_) + ((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(5,3) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_));
    (*this)(5,4) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(5,5) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jG::Type_fr_base0_X_fr_jG()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jG& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_base0_X_fr_jG::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jD_;
    Scalar s_q_jB_;
    Scalar s_q_jF_;
    Scalar s_q_jE_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jD_;
    Scalar c_q_jE_;
    Scalar c_q_jF_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jF_ = TRAIT::sin( q(JF));
    s_q_jE_ = TRAIT::sin( q(JE));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jF_ = TRAIT::cos( q(JF));
    
    (*this)(0,0) = ((((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + (((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,1) = ((((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(0,2) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,0) = (((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + (((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(1,1) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(1,2) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,0) = ((((- c_q_jB_ *  c_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + ((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(2,1) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(2,2) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(3,0) = (((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_) + (((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + (((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_)) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((- 0.081 *  s_q_jA_) *  c_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(3,1) = (((((((( 0.081 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + (((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + (((((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + ((((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) + (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_)) + (((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,2) = ((((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  c_q_jF_));
    (*this)(3,3) = (((((((( 1.0 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + ((((((( 1.0 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 1.0 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(3,4) = ((((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(3,5) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,0) = ((((((((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + ((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_)) + ((((((- 0.081 *  c_q_jA_) *  s_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + (((( 0.081 *  c_q_jA_) *  c_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(4,1) = (((((((( 0.081 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((- 0.081 *  c_q_jA_) *  s_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + (((((((((- 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + ((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_)) + ((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(4,2) = (((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,3) = ((((((( 1.0 *  c_q_jA_) *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + ((((((( 1.0 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 1.0 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(4,4) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(4,5) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,0) = ((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_) + ((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((( 0.081 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_));
    (*this)(5,1) = ((((((( 0.081 *  c_q_jB_) *  c_q_jD_) + ((( 0.081 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + ((((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.081 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  s_q_jE_)) + ((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(5,2) = (((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  c_q_jF_));
    (*this)(5,3) = ((((- c_q_jB_ *  c_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + (((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(5,4) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(5,5) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link2_X_fr_link1::Type_fr_link2_X_fr_link1()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = - 0.2025;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link2_X_fr_link1& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link2_X_fr_link1::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar c_q_jB_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jB_ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = - c_q_jB_;
    (*this)(0,2) =  s_q_jB_;
    (*this)(1,0) =  s_q_jB_;
    (*this)(1,2) =  c_q_jB_;
    (*this)(3,1) = (- 0.2025 *  c_q_jB_);
    (*this)(3,3) = - c_q_jB_;
    (*this)(3,5) =  s_q_jB_;
    (*this)(4,1) = ( 0.2025 *  s_q_jB_);
    (*this)(4,3) =  s_q_jB_;
    (*this)(4,5) =  c_q_jB_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link1_X_fr_link2::Type_fr_link1_X_fr_link2()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = - 0.2025;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link1_X_fr_link2& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link1_X_fr_link2::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar c_q_jB_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jB_ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = - c_q_jB_;
    (*this)(0,1) =  s_q_jB_;
    (*this)(2,0) =  s_q_jB_;
    (*this)(2,1) =  c_q_jB_;
    (*this)(3,3) = - c_q_jB_;
    (*this)(3,4) =  s_q_jB_;
    (*this)(4,0) = (- 0.2025 *  c_q_jB_);
    (*this)(4,1) = ( 0.2025 *  s_q_jB_);
    (*this)(5,3) =  s_q_jB_;
    (*this)(5,4) =  c_q_jB_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link3_X_fr_link2::Type_fr_link3_X_fr_link2()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,1) = 0;
    (*this)(3,4) = 0;
    (*this)(4,1) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link3_X_fr_link2& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link3_X_fr_link2::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar c_q_jC_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    c_q_jC_ = TRAIT::cos( q(JC));
    
    (*this)(0,0) = - c_q_jC_;
    (*this)(0,2) =  s_q_jC_;
    (*this)(1,0) =  s_q_jC_;
    (*this)(1,2) =  c_q_jC_;
    (*this)(3,0) = ( 0.2045 *  s_q_jC_);
    (*this)(3,2) = ( 0.2045 *  c_q_jC_);
    (*this)(3,3) = - c_q_jC_;
    (*this)(3,5) =  s_q_jC_;
    (*this)(4,0) = ( 0.2045 *  c_q_jC_);
    (*this)(4,2) = (- 0.2045 *  s_q_jC_);
    (*this)(4,3) =  s_q_jC_;
    (*this)(4,5) =  c_q_jC_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link2_X_fr_link3::Type_fr_link2_X_fr_link3()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link2_X_fr_link3& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link2_X_fr_link3::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar c_q_jC_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    c_q_jC_ = TRAIT::cos( q(JC));
    
    (*this)(0,0) = - c_q_jC_;
    (*this)(0,1) =  s_q_jC_;
    (*this)(2,0) =  s_q_jC_;
    (*this)(2,1) =  c_q_jC_;
    (*this)(3,0) = ( 0.2045 *  s_q_jC_);
    (*this)(3,1) = ( 0.2045 *  c_q_jC_);
    (*this)(3,3) = - c_q_jC_;
    (*this)(3,4) =  s_q_jC_;
    (*this)(5,0) = ( 0.2045 *  c_q_jC_);
    (*this)(5,1) = (- 0.2045 *  s_q_jC_);
    (*this)(5,3) =  s_q_jC_;
    (*this)(5,4) =  c_q_jC_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link4_X_fr_link3::Type_fr_link4_X_fr_link3()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0.2155;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link4_X_fr_link3& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link4_X_fr_link3::update(const JState& q) {
    Scalar s_q_jD_;
    Scalar c_q_jD_;
    
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jD_ = TRAIT::cos( q(JD));
    
    (*this)(0,0) =  c_q_jD_;
    (*this)(0,2) =  s_q_jD_;
    (*this)(1,0) = - s_q_jD_;
    (*this)(1,2) =  c_q_jD_;
    (*this)(3,1) = ( 0.2155 *  c_q_jD_);
    (*this)(3,3) =  c_q_jD_;
    (*this)(3,5) =  s_q_jD_;
    (*this)(4,1) = (- 0.2155 *  s_q_jD_);
    (*this)(4,3) = - s_q_jD_;
    (*this)(4,5) =  c_q_jD_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link3_X_fr_link4::Type_fr_link3_X_fr_link4()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0.2155;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link3_X_fr_link4& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link3_X_fr_link4::update(const JState& q) {
    Scalar s_q_jD_;
    Scalar c_q_jD_;
    
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jD_ = TRAIT::cos( q(JD));
    
    (*this)(0,0) =  c_q_jD_;
    (*this)(0,1) = - s_q_jD_;
    (*this)(2,0) =  s_q_jD_;
    (*this)(2,1) =  c_q_jD_;
    (*this)(3,3) =  c_q_jD_;
    (*this)(3,4) = - s_q_jD_;
    (*this)(4,0) = ( 0.2155 *  c_q_jD_);
    (*this)(4,1) = (- 0.2155 *  s_q_jD_);
    (*this)(5,3) =  s_q_jD_;
    (*this)(5,4) =  c_q_jD_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link5_X_fr_link4::Type_fr_link5_X_fr_link4()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,1) = 0;
    (*this)(3,4) = 0;
    (*this)(4,1) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link5_X_fr_link4& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link5_X_fr_link4::update(const JState& q) {
    Scalar s_q_jE_;
    Scalar c_q_jE_;
    
    s_q_jE_ = TRAIT::sin( q(JE));
    c_q_jE_ = TRAIT::cos( q(JE));
    
    (*this)(0,0) = - c_q_jE_;
    (*this)(0,2) =  s_q_jE_;
    (*this)(1,0) =  s_q_jE_;
    (*this)(1,2) =  c_q_jE_;
    (*this)(3,0) = ( 0.1845 *  s_q_jE_);
    (*this)(3,2) = ( 0.1845 *  c_q_jE_);
    (*this)(3,3) = - c_q_jE_;
    (*this)(3,5) =  s_q_jE_;
    (*this)(4,0) = ( 0.1845 *  c_q_jE_);
    (*this)(4,2) = (- 0.1845 *  s_q_jE_);
    (*this)(4,3) =  s_q_jE_;
    (*this)(4,5) =  c_q_jE_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link4_X_fr_link5::Type_fr_link4_X_fr_link5()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link4_X_fr_link5& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link4_X_fr_link5::update(const JState& q) {
    Scalar s_q_jE_;
    Scalar c_q_jE_;
    
    s_q_jE_ = TRAIT::sin( q(JE));
    c_q_jE_ = TRAIT::cos( q(JE));
    
    (*this)(0,0) = - c_q_jE_;
    (*this)(0,1) =  s_q_jE_;
    (*this)(2,0) =  s_q_jE_;
    (*this)(2,1) =  c_q_jE_;
    (*this)(3,0) = ( 0.1845 *  s_q_jE_);
    (*this)(3,1) = ( 0.1845 *  c_q_jE_);
    (*this)(3,3) = - c_q_jE_;
    (*this)(3,4) =  s_q_jE_;
    (*this)(5,0) = ( 0.1845 *  c_q_jE_);
    (*this)(5,1) = (- 0.1845 *  s_q_jE_);
    (*this)(5,3) =  s_q_jE_;
    (*this)(5,4) =  c_q_jE_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link6_X_fr_link5::Type_fr_link6_X_fr_link5()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0.2155;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link6_X_fr_link5& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link6_X_fr_link5::update(const JState& q) {
    Scalar s_q_jF_;
    Scalar c_q_jF_;
    
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jF_ = TRAIT::cos( q(JF));
    
    (*this)(0,0) =  c_q_jF_;
    (*this)(0,2) =  s_q_jF_;
    (*this)(1,0) = - s_q_jF_;
    (*this)(1,2) =  c_q_jF_;
    (*this)(3,1) = ( 0.2155 *  c_q_jF_);
    (*this)(3,3) =  c_q_jF_;
    (*this)(3,5) =  s_q_jF_;
    (*this)(4,1) = (- 0.2155 *  s_q_jF_);
    (*this)(4,3) = - s_q_jF_;
    (*this)(4,5) =  c_q_jF_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link5_X_fr_link6::Type_fr_link5_X_fr_link6()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0.2155;
    (*this)(3,5) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1.0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link5_X_fr_link6& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link5_X_fr_link6::update(const JState& q) {
    Scalar s_q_jF_;
    Scalar c_q_jF_;
    
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jF_ = TRAIT::cos( q(JF));
    
    (*this)(0,0) =  c_q_jF_;
    (*this)(0,1) = - s_q_jF_;
    (*this)(2,0) =  s_q_jF_;
    (*this)(2,1) =  c_q_jF_;
    (*this)(3,3) =  c_q_jF_;
    (*this)(3,4) = - s_q_jF_;
    (*this)(4,0) = ( 0.2155 *  c_q_jF_);
    (*this)(4,1) = (- 0.2155 *  s_q_jF_);
    (*this)(5,3) =  s_q_jF_;
    (*this)(5,4) =  c_q_jF_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link7_X_fr_link6::Type_fr_link7_X_fr_link6()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,1) = 0;
    (*this)(3,4) = 0;
    (*this)(4,1) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link7_X_fr_link6& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link7_X_fr_link6::update(const JState& q) {
    Scalar s_q_jG_;
    Scalar c_q_jG_;
    
    s_q_jG_ = TRAIT::sin( q(JG));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = - c_q_jG_;
    (*this)(0,2) =  s_q_jG_;
    (*this)(1,0) =  s_q_jG_;
    (*this)(1,2) =  c_q_jG_;
    (*this)(3,0) = ( 0.081 *  s_q_jG_);
    (*this)(3,2) = ( 0.081 *  c_q_jG_);
    (*this)(3,3) = - c_q_jG_;
    (*this)(3,5) =  s_q_jG_;
    (*this)(4,0) = ( 0.081 *  c_q_jG_);
    (*this)(4,2) = (- 0.081 *  s_q_jG_);
    (*this)(4,3) =  s_q_jG_;
    (*this)(4,5) =  c_q_jG_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link6_X_fr_link7::Type_fr_link6_X_fr_link7()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link6_X_fr_link7& iit::Kuka::tpl::MotionTransforms<TRAIT>::Type_fr_link6_X_fr_link7::update(const JState& q) {
    Scalar s_q_jG_;
    Scalar c_q_jG_;
    
    s_q_jG_ = TRAIT::sin( q(JG));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = - c_q_jG_;
    (*this)(0,1) =  s_q_jG_;
    (*this)(2,0) =  s_q_jG_;
    (*this)(2,1) =  c_q_jG_;
    (*this)(3,0) = ( 0.081 *  s_q_jG_);
    (*this)(3,1) = ( 0.081 *  c_q_jG_);
    (*this)(3,3) = - c_q_jG_;
    (*this)(3,4) =  s_q_jG_;
    (*this)(5,0) = ( 0.081 *  c_q_jG_);
    (*this)(5,1) = (- 0.081 *  s_q_jG_);
    (*this)(5,3) =  s_q_jG_;
    (*this)(5,4) =  c_q_jG_;
    return *this;
}

template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link1::Type_fr_base0_X_fr_link1()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link1& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link1::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar c_q_jA_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    c_q_jA_ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  c_q_jA_;
    (*this)(0,1) = - s_q_jA_;
    (*this)(0,3) = (- 0.1575 *  s_q_jA_);
    (*this)(0,4) = (- 0.1575 *  c_q_jA_);
    (*this)(1,0) =  s_q_jA_;
    (*this)(1,1) =  c_q_jA_;
    (*this)(1,3) = ( 0.1575 *  c_q_jA_);
    (*this)(1,4) = (- 0.1575 *  s_q_jA_);
    (*this)(3,3) =  c_q_jA_;
    (*this)(3,4) = - s_q_jA_;
    (*this)(4,3) =  s_q_jA_;
    (*this)(4,4) =  c_q_jA_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link2::Type_fr_base0_X_fr_link2()
{
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link2& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link2::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar s_q_jA_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jA_ = TRAIT::sin( q(JA));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = (- c_q_jA_ *  c_q_jB_);
    (*this)(0,1) = ( c_q_jA_ *  s_q_jB_);
    (*this)(0,2) = - s_q_jA_;
    (*this)(0,3) = (( 0.36 *  s_q_jA_) *  c_q_jB_);
    (*this)(0,4) = ((- 0.36 *  s_q_jA_) *  s_q_jB_);
    (*this)(0,5) = (- 0.36 *  c_q_jA_);
    (*this)(1,0) = (- s_q_jA_ *  c_q_jB_);
    (*this)(1,1) = ( s_q_jA_ *  s_q_jB_);
    (*this)(1,2) =  c_q_jA_;
    (*this)(1,3) = ((- 0.36 *  c_q_jA_) *  c_q_jB_);
    (*this)(1,4) = (( 0.36 *  c_q_jA_) *  s_q_jB_);
    (*this)(1,5) = (- 0.36 *  s_q_jA_);
    (*this)(2,0) =  s_q_jB_;
    (*this)(2,1) =  c_q_jB_;
    (*this)(3,3) = (- c_q_jA_ *  c_q_jB_);
    (*this)(3,4) = ( c_q_jA_ *  s_q_jB_);
    (*this)(3,5) = - s_q_jA_;
    (*this)(4,3) = (- s_q_jA_ *  c_q_jB_);
    (*this)(4,4) = ( s_q_jA_ *  s_q_jB_);
    (*this)(4,5) =  c_q_jA_;
    (*this)(5,3) =  s_q_jB_;
    (*this)(5,4) =  c_q_jB_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link3::Type_fr_base0_X_fr_link3()
{
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link3& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link3::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jB_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    
    (*this)(0,0) = ((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_));
    (*this)(0,1) = (((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_));
    (*this)(0,2) = ( c_q_jA_ *  s_q_jB_);
    (*this)(0,3) = ((((- 0.36 *  c_q_jA_) - (( 0.2045 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.2045 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(0,4) = (((( 0.2045 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.36 *  c_q_jA_) - (( 0.2045 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(0,5) = ((- 0.36 *  s_q_jA_) *  s_q_jB_);
    (*this)(1,0) = (( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_));
    (*this)(1,1) = (( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_));
    (*this)(1,2) = ( s_q_jA_ *  s_q_jB_);
    (*this)(1,3) = ((((- 0.36 *  s_q_jA_) - (( 0.2045 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.2045 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(1,4) = ((((- 0.2045 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.36 *  s_q_jA_) - (( 0.2045 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(1,5) = (( 0.36 *  c_q_jA_) *  s_q_jB_);
    (*this)(2,0) = (- s_q_jB_ *  c_q_jC_);
    (*this)(2,1) = ( s_q_jB_ *  s_q_jC_);
    (*this)(2,2) =  c_q_jB_;
    (*this)(2,3) = (( 0.2045 *  s_q_jB_) *  s_q_jC_);
    (*this)(2,4) = (( 0.2045 *  s_q_jB_) *  c_q_jC_);
    (*this)(3,3) = ((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_));
    (*this)(3,4) = (((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_));
    (*this)(3,5) = ( c_q_jA_ *  s_q_jB_);
    (*this)(4,3) = (( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_));
    (*this)(4,4) = (( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_));
    (*this)(4,5) = ( s_q_jA_ *  s_q_jB_);
    (*this)(5,3) = (- s_q_jB_ *  c_q_jC_);
    (*this)(5,4) = ( s_q_jB_ *  s_q_jC_);
    (*this)(5,5) =  c_q_jB_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link4::Type_fr_base0_X_fr_link4()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link4& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link4::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jD_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jD_ = TRAIT::cos( q(JD));
    
    (*this)(0,0) = ((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(0,1) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(0,2) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(0,3) = (((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_));
    (*this)(0,4) = ((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(0,5) = ((((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(1,0) = ((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(1,1) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,2) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(1,3) = (((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_));
    (*this)(1,4) = (((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(1,5) = (((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(2,0) = (( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_));
    (*this)(2,1) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(2,2) = (- s_q_jB_ *  s_q_jC_);
    (*this)(2,3) = ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_);
    (*this)(2,4) = (((- 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_);
    (*this)(2,5) = ((- 0.42 *  s_q_jB_) *  c_q_jC_);
    (*this)(3,3) = ((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(3,4) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(3,5) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(4,3) = ((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(4,4) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(4,5) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(5,3) = (( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_));
    (*this)(5,4) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(5,5) = (- s_q_jB_ *  s_q_jC_);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link5::Type_fr_base0_X_fr_link5()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link5& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link5::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jE_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar c_q_jC_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jD_;
    Scalar c_q_jE_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jE_ = TRAIT::cos( q(JE));
    
    (*this)(0,0) = (((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,1) = ((((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(0,2) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(0,3) = ((((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.1845 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.1845 *  s_q_jA_) *  c_q_jC_) + ((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,4) = (((((((- 0.1845 *  s_q_jA_) *  c_q_jC_) - ((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.1845 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,5) = ((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(1,0) = ((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(1,1) = (((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  c_q_jE_));
    (*this)(1,2) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,3) = ((((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.1845 *  c_q_jA_) *  s_q_jC_) + ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.1845 *  c_q_jA_) *  c_q_jC_) + ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(1,4) = ((((((( 0.1845 *  c_q_jA_) *  c_q_jC_) - ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.1845 *  c_q_jA_) *  s_q_jC_) + ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(1,5) = (((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(2,0) = ((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_));
    (*this)(2,1) = (((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(2,2) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(2,3) = ((((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.1845 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.1845 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((- 0.1845 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_));
    (*this)(2,4) = ((((( 0.1845 *  s_q_jB_) *  s_q_jC_) + ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  s_q_jE_) + (((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.1845 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.1845 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(2,5) = (((- 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_);
    (*this)(3,3) = (((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,4) = ((((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(3,5) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(4,3) = ((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(4,4) = (((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  c_q_jE_));
    (*this)(4,5) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(5,3) = ((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_));
    (*this)(5,4) = (((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(5,5) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link6::Type_fr_base0_X_fr_link6()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link6& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link6::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jD_;
    Scalar s_q_jF_;
    Scalar s_q_jE_;
    Scalar c_q_jA_;
    Scalar c_q_jD_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jF_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jF_ = TRAIT::sin( q(JF));
    s_q_jE_ = TRAIT::sin( q(JE));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jF_ = TRAIT::cos( q(JF));
    
    (*this)(0,0) = ((((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,1) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(0,2) = ((((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(0,3) = (((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + ((((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + ((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,4) = ((((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  c_q_jF_));
    (*this)(0,5) = ((((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(1,0) = ((((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(1,1) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,2) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(1,3) = (((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + (((((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + ((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(1,4) = (((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,5) = (((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(2,0) = (((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + (((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(2,1) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,2) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(2,3) = ((((((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((( 0.4 *  c_q_jB_) *  s_q_jD_) - ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_));
    (*this)(2,4) = (((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  c_q_jF_));
    (*this)(2,5) = (((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  s_q_jE_) + ((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,3) = ((((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(3,4) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(3,5) = (((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(4,3) = ((((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(4,4) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,5) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(5,3) = (((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + (((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(5,4) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,5) = ((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link7::Type_fr_base0_X_fr_link7()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link7& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_link7::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jE_;
    Scalar s_q_jG_;
    Scalar s_q_jF_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jD_;
    Scalar c_q_jF_;
    Scalar c_q_jG_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jG_ = TRAIT::sin( q(JG));
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jF_ = TRAIT::cos( q(JF));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,1) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(0,2) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(0,3) = ((((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.081 *  s_q_jA_) *  c_q_jC_) + ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  s_q_jA_) *  c_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,4) = (((((((((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) + (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.081 *  s_q_jA_) *  c_q_jC_) + ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,5) = ((((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  c_q_jF_));
    (*this)(1,0) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,1) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,2) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,3) = ((((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.081 *  c_q_jA_) *  s_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((( 0.081 *  c_q_jA_) *  c_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((((- 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  c_q_jA_) *  c_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,4) = (((((((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  c_q_jA_) *  s_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.081 *  c_q_jA_) *  s_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,5) = (((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,0) = ((((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((( s_q_jB_ *  s_q_jC_) *  s_q_jE_) + ((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(2,1) = ((((((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + (((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(2,2) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,3) = ((((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.081 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.081 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((( 0.081 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 0.4 *  s_q_jB_) *  s_q_jC_) + ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  c_q_jG_));
    (*this)(2,4) = ((((((((- 0.081 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + (((( 0.081 *  c_q_jB_) *  s_q_jD_) - ((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_)) + (((((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.081 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.081 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(2,5) = (((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  c_q_jF_));
    (*this)(3,3) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,4) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(3,5) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,3) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,4) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(4,5) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,3) = ((((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((( s_q_jB_ *  s_q_jC_) *  s_q_jE_) + ((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(5,4) = ((((((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + (((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(5,5) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_ee::Type_fr_base0_X_fr_ee()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_ee& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_ee::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jE_;
    Scalar s_q_jG_;
    Scalar s_q_jF_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jD_;
    Scalar c_q_jF_;
    Scalar c_q_jG_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jG_ = TRAIT::sin( q(JG));
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jF_ = TRAIT::cos( q(JF));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,1) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(0,2) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(0,3) = ((((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.241 *  s_q_jA_) *  c_q_jC_) + ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  s_q_jA_) *  c_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,4) = (((((((((( 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) + (( 0.241 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.241 *  s_q_jA_) *  c_q_jC_) + ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,5) = ((((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  c_q_jF_));
    (*this)(1,0) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,1) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,2) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,3) = ((((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.241 *  c_q_jA_) *  s_q_jC_) - ((( 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((( 0.241 *  c_q_jA_) *  c_q_jC_) - ((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((((- 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_) - ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  c_q_jA_) *  c_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,4) = (((((((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  c_q_jA_) *  s_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.241 *  c_q_jA_) *  s_q_jC_) - ((( 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,5) = (((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,0) = ((((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((( s_q_jB_ *  s_q_jC_) *  s_q_jE_) + ((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(2,1) = ((((((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + (((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(2,2) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,3) = ((((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.241 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.241 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.241 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((( 0.241 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.241 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 0.4 *  s_q_jB_) *  s_q_jC_) + ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  c_q_jG_));
    (*this)(2,4) = ((((((((- 0.241 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + (((( 0.241 *  c_q_jB_) *  s_q_jD_) - ((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_)) + (((((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.241 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.241 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.241 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(2,5) = (((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  c_q_jF_));
    (*this)(3,3) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,4) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(3,5) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,3) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,4) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(4,5) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,3) = ((((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((( s_q_jB_ *  s_q_jC_) *  s_q_jE_) + ((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(5,4) = ((((((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + (((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(5,5) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link1_X_fr_base0::Type_fr_link1_X_fr_base0()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link1_X_fr_base0& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link1_X_fr_base0::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar c_q_jA_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    c_q_jA_ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  c_q_jA_;
    (*this)(0,1) =  s_q_jA_;
    (*this)(0,3) = (- 0.1575 *  s_q_jA_);
    (*this)(0,4) = ( 0.1575 *  c_q_jA_);
    (*this)(1,0) = - s_q_jA_;
    (*this)(1,1) =  c_q_jA_;
    (*this)(1,3) = (- 0.1575 *  c_q_jA_);
    (*this)(1,4) = (- 0.1575 *  s_q_jA_);
    (*this)(3,3) =  c_q_jA_;
    (*this)(3,4) =  s_q_jA_;
    (*this)(4,3) = - s_q_jA_;
    (*this)(4,4) =  c_q_jA_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link2_X_fr_base0::Type_fr_link2_X_fr_base0()
{
    (*this)(0,5) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link2_X_fr_base0& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link2_X_fr_base0::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = (- c_q_jA_ *  c_q_jB_);
    (*this)(0,1) = (- s_q_jA_ *  c_q_jB_);
    (*this)(0,2) =  s_q_jB_;
    (*this)(0,3) = (( 0.36 *  s_q_jA_) *  c_q_jB_);
    (*this)(0,4) = ((- 0.36 *  c_q_jA_) *  c_q_jB_);
    (*this)(1,0) = ( c_q_jA_ *  s_q_jB_);
    (*this)(1,1) = ( s_q_jA_ *  s_q_jB_);
    (*this)(1,2) =  c_q_jB_;
    (*this)(1,3) = ((- 0.36 *  s_q_jA_) *  s_q_jB_);
    (*this)(1,4) = (( 0.36 *  c_q_jA_) *  s_q_jB_);
    (*this)(2,0) = - s_q_jA_;
    (*this)(2,1) =  c_q_jA_;
    (*this)(2,3) = (- 0.36 *  c_q_jA_);
    (*this)(2,4) = (- 0.36 *  s_q_jA_);
    (*this)(3,3) = (- c_q_jA_ *  c_q_jB_);
    (*this)(3,4) = (- s_q_jA_ *  c_q_jB_);
    (*this)(3,5) =  s_q_jB_;
    (*this)(4,3) = ( c_q_jA_ *  s_q_jB_);
    (*this)(4,4) = ( s_q_jA_ *  s_q_jB_);
    (*this)(4,5) =  c_q_jB_;
    (*this)(5,3) = - s_q_jA_;
    (*this)(5,4) =  c_q_jA_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link3_X_fr_base0::Type_fr_link3_X_fr_base0()
{
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link3_X_fr_base0& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link3_X_fr_base0::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jB_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    
    (*this)(0,0) = ((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_));
    (*this)(0,1) = (( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_));
    (*this)(0,2) = (- s_q_jB_ *  c_q_jC_);
    (*this)(0,3) = ((((- 0.36 *  c_q_jA_) - (( 0.2045 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.2045 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(0,4) = ((((- 0.36 *  s_q_jA_) - (( 0.2045 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.2045 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(0,5) = (( 0.2045 *  s_q_jB_) *  s_q_jC_);
    (*this)(1,0) = (((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_));
    (*this)(1,1) = (( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_));
    (*this)(1,2) = ( s_q_jB_ *  s_q_jC_);
    (*this)(1,3) = (((( 0.2045 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.36 *  c_q_jA_) - (( 0.2045 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(1,4) = ((((- 0.2045 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.36 *  s_q_jA_) - (( 0.2045 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(1,5) = (( 0.2045 *  s_q_jB_) *  c_q_jC_);
    (*this)(2,0) = ( c_q_jA_ *  s_q_jB_);
    (*this)(2,1) = ( s_q_jA_ *  s_q_jB_);
    (*this)(2,2) =  c_q_jB_;
    (*this)(2,3) = ((- 0.36 *  s_q_jA_) *  s_q_jB_);
    (*this)(2,4) = (( 0.36 *  c_q_jA_) *  s_q_jB_);
    (*this)(3,3) = ((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_));
    (*this)(3,4) = (( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_));
    (*this)(3,5) = (- s_q_jB_ *  c_q_jC_);
    (*this)(4,3) = (((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_));
    (*this)(4,4) = (( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_));
    (*this)(4,5) = ( s_q_jB_ *  s_q_jC_);
    (*this)(5,3) = ( c_q_jA_ *  s_q_jB_);
    (*this)(5,4) = ( s_q_jA_ *  s_q_jB_);
    (*this)(5,5) =  c_q_jB_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link4_X_fr_base0::Type_fr_link4_X_fr_base0()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link4_X_fr_base0& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link4_X_fr_base0::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jD_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jD_ = TRAIT::cos( q(JD));
    
    (*this)(0,0) = ((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(0,1) = ((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(0,2) = (( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_));
    (*this)(0,3) = (((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_));
    (*this)(0,4) = (((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_));
    (*this)(0,5) = ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_);
    (*this)(1,0) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,1) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,2) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(1,3) = ((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(1,4) = (((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(1,5) = (((- 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_);
    (*this)(2,0) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(2,1) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(2,2) = (- s_q_jB_ *  s_q_jC_);
    (*this)(2,3) = ((((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(2,4) = (((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(2,5) = ((- 0.42 *  s_q_jB_) *  c_q_jC_);
    (*this)(3,3) = ((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(3,4) = ((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(3,5) = (( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_));
    (*this)(4,3) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(4,4) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(4,5) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(5,3) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(5,4) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(5,5) = (- s_q_jB_ *  s_q_jC_);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link5_X_fr_base0::Type_fr_link5_X_fr_base0()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link5_X_fr_base0& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link5_X_fr_base0::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jE_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar c_q_jC_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jD_;
    Scalar c_q_jE_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jE_ = TRAIT::cos( q(JE));
    
    (*this)(0,0) = (((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,1) = ((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,2) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_));
    (*this)(0,3) = ((((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.1845 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.1845 *  s_q_jA_) *  c_q_jC_) + ((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,4) = ((((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.1845 *  c_q_jA_) *  s_q_jC_) + ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.1845 *  c_q_jA_) *  c_q_jC_) + ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,5) = ((((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.1845 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.1845 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((- 0.1845 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_));
    (*this)(1,0) = ((((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(1,1) = (((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  c_q_jE_));
    (*this)(1,2) = ((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(1,3) = (((((((- 0.1845 *  s_q_jA_) *  c_q_jC_) - ((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.1845 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(1,4) = ((((((( 0.1845 *  c_q_jA_) *  c_q_jC_) - ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.1845 *  c_q_jA_) *  s_q_jC_) + ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(1,5) = ((((( 0.1845 *  s_q_jB_) *  s_q_jC_) + ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  s_q_jE_) + (((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.1845 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.1845 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(2,0) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(2,1) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(2,2) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(2,3) = ((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(2,4) = (((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(2,5) = (((- 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_);
    (*this)(3,3) = (((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,4) = ((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,5) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_));
    (*this)(4,3) = ((((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(4,4) = (((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  c_q_jE_));
    (*this)(4,5) = ((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(5,3) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(5,4) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(5,5) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link6_X_fr_base0::Type_fr_link6_X_fr_base0()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link6_X_fr_base0& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link6_X_fr_base0::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jD_;
    Scalar s_q_jF_;
    Scalar s_q_jE_;
    Scalar c_q_jA_;
    Scalar c_q_jD_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jF_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jF_ = TRAIT::sin( q(JF));
    s_q_jE_ = TRAIT::sin( q(JE));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jF_ = TRAIT::cos( q(JF));
    
    (*this)(0,0) = ((((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,1) = ((((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,2) = ((((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + ((((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,3) = (((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + ((((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + ((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,4) = (((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + (((((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + ((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,5) = ((((((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((( 0.4 *  c_q_jB_) *  s_q_jD_) - ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_));
    (*this)(1,0) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,1) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,2) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,3) = ((((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  c_q_jF_));
    (*this)(1,4) = (((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,5) = (((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  c_q_jF_));
    (*this)(2,0) = (((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(2,1) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(2,2) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(2,3) = ((((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(2,4) = (((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(2,5) = (((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  s_q_jE_) + ((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,3) = ((((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(3,4) = ((((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(3,5) = ((((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + ((((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(4,3) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,4) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,5) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,3) = (((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(5,4) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(5,5) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link7_X_fr_base0::Type_fr_link7_X_fr_base0()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link7_X_fr_base0& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link7_X_fr_base0::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jE_;
    Scalar s_q_jG_;
    Scalar s_q_jF_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jD_;
    Scalar c_q_jF_;
    Scalar c_q_jG_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jG_ = TRAIT::sin( q(JG));
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jF_ = TRAIT::cos( q(JF));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,1) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,2) = (((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_) + (((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,3) = ((((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.081 *  s_q_jA_) *  c_q_jC_) + ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  s_q_jA_) *  c_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,4) = ((((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.081 *  c_q_jA_) *  s_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((( 0.081 *  c_q_jA_) *  c_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((((- 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  c_q_jA_) *  c_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,5) = ((((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.081 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.081 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((( 0.081 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 0.4 *  s_q_jB_) *  s_q_jC_) + ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,0) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,1) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,2) = (((((((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,3) = (((((((((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) + (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.081 *  s_q_jA_) *  c_q_jC_) + ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,4) = (((((((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  c_q_jA_) *  s_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.081 *  c_q_jA_) *  s_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,5) = ((((((((- 0.081 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + (((( 0.081 *  c_q_jB_) *  s_q_jD_) - ((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_)) + (((((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.081 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.081 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(2,0) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,1) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,2) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,3) = ((((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  c_q_jF_));
    (*this)(2,4) = (((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,5) = (((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  c_q_jF_));
    (*this)(3,3) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,4) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,5) = (((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_) + (((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,3) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(4,4) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(4,5) = (((((((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(5,3) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,4) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,5) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_ee_X_fr_base0::Type_fr_ee_X_fr_base0()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_ee_X_fr_base0& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_ee_X_fr_base0::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jE_;
    Scalar s_q_jG_;
    Scalar s_q_jF_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jD_;
    Scalar c_q_jF_;
    Scalar c_q_jG_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jG_ = TRAIT::sin( q(JG));
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jF_ = TRAIT::cos( q(JF));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,1) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,2) = (((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_) + (((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,3) = ((((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.241 *  s_q_jA_) *  c_q_jC_) + ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  s_q_jA_) *  c_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,4) = ((((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.241 *  c_q_jA_) *  s_q_jC_) - ((( 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((( 0.241 *  c_q_jA_) *  c_q_jC_) - ((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((((- 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_) - ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  c_q_jA_) *  c_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,5) = ((((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.241 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.241 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.241 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((( 0.241 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.241 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 0.4 *  s_q_jB_) *  s_q_jC_) + ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,0) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,1) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,2) = (((((((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,3) = (((((((((( 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) + (( 0.241 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + ((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + (((((( 0.241 *  s_q_jA_) *  c_q_jC_) + ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.241 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,4) = (((((((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + ((((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  s_q_jE_)) + (((((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  c_q_jA_) *  s_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + ((((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((((((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + ((((((- 0.241 *  c_q_jA_) *  s_q_jC_) - ((( 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) + (((((((- 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.241 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,5) = ((((((((- 0.241 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + (((( 0.241 *  c_q_jB_) *  s_q_jD_) - ((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_)) + (((((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_)) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_)) *  s_q_jG_) + (((((((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  c_q_jE_) + (((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  s_q_jE_)) + ((((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.241 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.241 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.241 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.241 *  c_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(2,0) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,1) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,2) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,3) = ((((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  c_q_jF_));
    (*this)(2,4) = (((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,5) = (((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  c_q_jF_));
    (*this)(3,3) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,4) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(3,5) = (((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_) + (((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(4,3) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(4,4) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(4,5) = (((((((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(5,3) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,4) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,5) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jA::Type_fr_base0_X_fr_jA()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = - 0.1575;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0.1575;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
    (*this)(3,4) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 1;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jA& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jB::Type_fr_base0_X_fr_jB()
{
    (*this)(0,1) = 0;
    (*this)(0,4) = 0;
    (*this)(1,1) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jB& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jB::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar c_q_jA_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    c_q_jA_ = TRAIT::cos( q(JA));
    
    (*this)(0,0) = - c_q_jA_;
    (*this)(0,2) = - s_q_jA_;
    (*this)(0,3) = ( 0.36 *  s_q_jA_);
    (*this)(0,5) = (- 0.36 *  c_q_jA_);
    (*this)(1,0) = - s_q_jA_;
    (*this)(1,2) =  c_q_jA_;
    (*this)(1,3) = (- 0.36 *  c_q_jA_);
    (*this)(1,5) = (- 0.36 *  s_q_jA_);
    (*this)(3,3) = - c_q_jA_;
    (*this)(3,5) = - s_q_jA_;
    (*this)(4,3) = - s_q_jA_;
    (*this)(4,5) =  c_q_jA_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jC::Type_fr_base0_X_fr_jC()
{
    (*this)(2,1) = 0;
    (*this)(2,3) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,4) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jC& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jC::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = ( c_q_jA_ *  c_q_jB_);
    (*this)(0,1) = - s_q_jA_;
    (*this)(0,2) = ( c_q_jA_ *  s_q_jB_);
    (*this)(0,3) = (((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.2045 *  s_q_jA_));
    (*this)(0,4) = (((- 0.2045 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_));
    (*this)(0,5) = ((- 0.36 *  s_q_jA_) *  s_q_jB_);
    (*this)(1,0) = ( s_q_jA_ *  c_q_jB_);
    (*this)(1,1) =  c_q_jA_;
    (*this)(1,2) = ( s_q_jA_ *  s_q_jB_);
    (*this)(1,3) = ((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.2045 *  c_q_jA_));
    (*this)(1,4) = (((- 0.2045 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_));
    (*this)(1,5) = (( 0.36 *  c_q_jA_) *  s_q_jB_);
    (*this)(2,0) = - s_q_jB_;
    (*this)(2,2) =  c_q_jB_;
    (*this)(2,4) = ( 0.2045 *  s_q_jB_);
    (*this)(3,3) = ( c_q_jA_ *  c_q_jB_);
    (*this)(3,4) = - s_q_jA_;
    (*this)(3,5) = ( c_q_jA_ *  s_q_jB_);
    (*this)(4,3) = ( s_q_jA_ *  c_q_jB_);
    (*this)(4,4) =  c_q_jA_;
    (*this)(4,5) = ( s_q_jA_ *  s_q_jB_);
    (*this)(5,3) = - s_q_jB_;
    (*this)(5,5) =  c_q_jB_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jD::Type_fr_base0_X_fr_jD()
{
    (*this)(2,4) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jD& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jD::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jB_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    
    (*this)(0,0) = ((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_));
    (*this)(0,1) = ( c_q_jA_ *  s_q_jB_);
    (*this)(0,2) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(0,3) = ((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(0,4) = ((- 0.36 *  s_q_jA_) *  s_q_jB_);
    (*this)(0,5) = ((((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(1,0) = (( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_));
    (*this)(1,1) = ( s_q_jA_ *  s_q_jB_);
    (*this)(1,2) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(1,3) = ((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(1,4) = (( 0.36 *  c_q_jA_) *  s_q_jB_);
    (*this)(1,5) = (((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(2,0) = (- s_q_jB_ *  c_q_jC_);
    (*this)(2,1) =  c_q_jB_;
    (*this)(2,2) = (- s_q_jB_ *  s_q_jC_);
    (*this)(2,3) = (( 0.42 *  s_q_jB_) *  s_q_jC_);
    (*this)(2,5) = ((- 0.42 *  s_q_jB_) *  c_q_jC_);
    (*this)(3,3) = ((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_));
    (*this)(3,4) = ( c_q_jA_ *  s_q_jB_);
    (*this)(3,5) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(4,3) = (( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_));
    (*this)(4,4) = ( s_q_jA_ *  s_q_jB_);
    (*this)(4,5) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(5,3) = (- s_q_jB_ *  c_q_jC_);
    (*this)(5,4) =  c_q_jB_;
    (*this)(5,5) = (- s_q_jB_ *  s_q_jC_);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jE::Type_fr_base0_X_fr_jE()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jE& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jE::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jD_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jD_ = TRAIT::cos( q(JD));
    
    (*this)(0,0) = ((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_));
    (*this)(0,1) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(0,2) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(0,3) = (((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.1845 *  s_q_jA_) *  c_q_jC_));
    (*this)(0,4) = (((((( 0.1845 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.1845 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + ((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(0,5) = ((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(1,0) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_));
    (*this)(1,1) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(1,2) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,3) = ((((((- 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.1845 *  c_q_jA_) *  c_q_jC_));
    (*this)(1,4) = (((((( 0.1845 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.1845 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.1845 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + ((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_));
    (*this)(1,5) = (((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(2,0) = ((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_));
    (*this)(2,1) = (- s_q_jB_ *  s_q_jC_);
    (*this)(2,2) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(2,3) = ((((- 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) - (( 0.1845 *  s_q_jB_) *  s_q_jC_));
    (*this)(2,4) = (((( 0.1845 *  c_q_jB_) *  s_q_jD_) - ((( 0.1845 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.42 *  s_q_jB_) *  c_q_jC_));
    (*this)(2,5) = (((- 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_);
    (*this)(3,3) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_));
    (*this)(3,4) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(3,5) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(4,3) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_));
    (*this)(4,4) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(4,5) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(5,3) = ((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_));
    (*this)(5,4) = (- s_q_jB_ *  s_q_jC_);
    (*this)(5,5) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jF::Type_fr_base0_X_fr_jF()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jF& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jF::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jE_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar c_q_jC_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jD_;
    Scalar c_q_jE_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jE_ = TRAIT::cos( q(JE));
    
    (*this)(0,0) = ((((( 1.0 *  s_q_jA_) *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,1) = ((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(0,2) = ((((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(0,3) = ((((((((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + ((((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,4) = ((((((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  c_q_jC_) + (((( 0.42 *  c_q_jA_) *  c_q_jB_) + ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(0,5) = ((((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(1,0) = (((((( 1.0 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(1,1) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,2) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(1,3) = ((((((((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  c_q_jC_) + (((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  c_q_jA_) *  s_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(1,4) = (((((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  c_q_jC_) + (((( 0.42 *  s_q_jA_) *  c_q_jB_) + ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  s_q_jD_) + ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_));
    (*this)(1,5) = (((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(2,0) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_));
    (*this)(2,1) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(2,2) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(2,3) = ((((((- 0.42 *  s_q_jB_) *  c_q_jC_) - ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  c_q_jE_));
    (*this)(2,4) = (((- 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_);
    (*this)(2,5) = (((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  s_q_jE_) + ((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,3) = (((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(3,4) = ((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(3,5) = (((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(4,3) = ((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(4,4) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(4,5) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(5,3) = ((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_));
    (*this)(5,4) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(5,5) = ((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jG::Type_fr_base0_X_fr_jG()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jG& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_base0_X_fr_jG::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jD_;
    Scalar s_q_jB_;
    Scalar s_q_jF_;
    Scalar s_q_jE_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jD_;
    Scalar c_q_jE_;
    Scalar c_q_jF_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jF_ = TRAIT::sin( q(JF));
    s_q_jE_ = TRAIT::sin( q(JE));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jF_ = TRAIT::cos( q(JF));
    
    (*this)(0,0) = (((((((( 1.0 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + ((((((( 1.0 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 1.0 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,1) = ((((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(0,2) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(0,3) = (((((((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_) + (((((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  s_q_jA_) - (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + (((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_)) + (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((- 0.081 *  s_q_jA_) *  c_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(0,4) = (((((((( 0.081 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + (((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + (((((((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + ((((( 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) + (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_)) + (((((( 0.4 *  s_q_jA_) *  c_q_jC_) + ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  c_q_jC_) + (((( 0.36 *  s_q_jA_) *  c_q_jB_) + ( 0.42 *  s_q_jA_)) *  s_q_jC_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,5) = ((((((((((- 0.36 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((((- 0.36 *  s_q_jA_) *  c_q_jB_) - ( 0.42 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  c_q_jA_) *  c_q_jB_) - ( 0.36 *  c_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_)) - (( 0.4 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  c_q_jA_) - (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((((( 0.36 *  c_q_jA_) + (( 0.42 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  s_q_jA_) + (( 0.36 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  c_q_jF_));
    (*this)(1,0) = ((((((( 1.0 *  c_q_jA_) *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + ((((((( 1.0 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 1.0 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(1,1) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(1,2) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,3) = ((((((((((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + ((( 0.42 *  c_q_jA_) + (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + ((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_)) + ((((((- 0.081 *  c_q_jA_) *  s_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + (((( 0.081 *  c_q_jA_) *  c_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(1,4) = (((((((( 0.081 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((- 0.081 *  c_q_jA_) *  s_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + (((((((((- 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_) - ((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + ((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_)) + ((((((- 0.4 *  c_q_jA_) *  c_q_jC_) + ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((((((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  c_q_jC_) + ((((- 0.36 *  c_q_jA_) *  c_q_jB_) - ( 0.42 *  c_q_jA_)) *  s_q_jC_)) + ((((- 0.4 *  c_q_jA_) *  s_q_jC_) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(1,5) = (((((((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((( 0.36 *  c_q_jA_) *  c_q_jB_) + ( 0.42 *  c_q_jA_)) *  c_q_jC_) + ((((- 0.42 *  s_q_jA_) *  c_q_jB_) - ( 0.36 *  s_q_jA_)) *  s_q_jC_)) *  c_q_jD_)) - ((( 0.4 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) + (( 0.4 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((((((- 0.4 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  s_q_jC_)) + (((- 0.36 *  s_q_jA_) - (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.36 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((( 0.36 *  s_q_jA_) + (( 0.42 *  s_q_jA_) *  c_q_jB_)) *  s_q_jC_) + (((- 0.42 *  c_q_jA_) - (( 0.36 *  c_q_jA_) *  c_q_jB_)) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,0) = ((((- c_q_jB_ *  c_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + (((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(2,1) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(2,2) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,3) = ((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  s_q_jF_) + ((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_)) + ((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  s_q_jE_)) + ((( 0.081 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_));
    (*this)(2,4) = ((((((( 0.081 *  c_q_jB_) *  c_q_jD_) + ((( 0.081 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + ((((((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - (( 0.081 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_) - ((( 0.081 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_)) + ((((- 0.4 *  s_q_jB_) *  s_q_jC_) - ((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_)) *  s_q_jE_)) + ((((( 0.42 *  s_q_jB_) *  c_q_jC_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) - (( 0.4 *  c_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(2,5) = (((((((( 0.42 *  s_q_jB_) *  s_q_jC_) *  c_q_jD_) + (( 0.4 *  s_q_jB_) *  s_q_jC_)) *  c_q_jE_) + (((((- 0.4 *  c_q_jB_) *  s_q_jD_) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) + (( 0.42 *  s_q_jB_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) - (((( 0.42 *  s_q_jB_) *  s_q_jC_) *  s_q_jD_) *  c_q_jF_));
    (*this)(3,3) = ((((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + (((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(3,4) = ((((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(3,5) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(4,3) = (((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + (((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(4,4) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(4,5) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(5,3) = ((((- c_q_jB_ *  c_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + ((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(5,4) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(5,5) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link2_X_fr_link1::Type_fr_link2_X_fr_link1()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.2025;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link2_X_fr_link1& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link2_X_fr_link1::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar c_q_jB_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jB_ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = - c_q_jB_;
    (*this)(0,2) =  s_q_jB_;
    (*this)(0,4) = (- 0.2025 *  c_q_jB_);
    (*this)(1,0) =  s_q_jB_;
    (*this)(1,2) =  c_q_jB_;
    (*this)(1,4) = ( 0.2025 *  s_q_jB_);
    (*this)(3,3) = - c_q_jB_;
    (*this)(3,5) =  s_q_jB_;
    (*this)(4,3) =  s_q_jB_;
    (*this)(4,5) =  c_q_jB_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link1_X_fr_link2::Type_fr_link1_X_fr_link2()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = - 0.2025;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link1_X_fr_link2& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link1_X_fr_link2::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar c_q_jB_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jB_ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = - c_q_jB_;
    (*this)(0,1) =  s_q_jB_;
    (*this)(1,3) = (- 0.2025 *  c_q_jB_);
    (*this)(1,4) = ( 0.2025 *  s_q_jB_);
    (*this)(2,0) =  s_q_jB_;
    (*this)(2,1) =  c_q_jB_;
    (*this)(3,3) = - c_q_jB_;
    (*this)(3,4) =  s_q_jB_;
    (*this)(5,3) =  s_q_jB_;
    (*this)(5,4) =  c_q_jB_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link3_X_fr_link2::Type_fr_link3_X_fr_link2()
{
    (*this)(0,1) = 0;
    (*this)(0,4) = 0;
    (*this)(1,1) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link3_X_fr_link2& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link3_X_fr_link2::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar c_q_jC_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    c_q_jC_ = TRAIT::cos( q(JC));
    
    (*this)(0,0) = - c_q_jC_;
    (*this)(0,2) =  s_q_jC_;
    (*this)(0,3) = ( 0.2045 *  s_q_jC_);
    (*this)(0,5) = ( 0.2045 *  c_q_jC_);
    (*this)(1,0) =  s_q_jC_;
    (*this)(1,2) =  c_q_jC_;
    (*this)(1,3) = ( 0.2045 *  c_q_jC_);
    (*this)(1,5) = (- 0.2045 *  s_q_jC_);
    (*this)(3,3) = - c_q_jC_;
    (*this)(3,5) =  s_q_jC_;
    (*this)(4,3) =  s_q_jC_;
    (*this)(4,5) =  c_q_jC_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link2_X_fr_link3::Type_fr_link2_X_fr_link3()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link2_X_fr_link3& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link2_X_fr_link3::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar c_q_jC_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    c_q_jC_ = TRAIT::cos( q(JC));
    
    (*this)(0,0) = - c_q_jC_;
    (*this)(0,1) =  s_q_jC_;
    (*this)(0,3) = ( 0.2045 *  s_q_jC_);
    (*this)(0,4) = ( 0.2045 *  c_q_jC_);
    (*this)(2,0) =  s_q_jC_;
    (*this)(2,1) =  c_q_jC_;
    (*this)(2,3) = ( 0.2045 *  c_q_jC_);
    (*this)(2,4) = (- 0.2045 *  s_q_jC_);
    (*this)(3,3) = - c_q_jC_;
    (*this)(3,4) =  s_q_jC_;
    (*this)(5,3) =  s_q_jC_;
    (*this)(5,4) =  c_q_jC_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link4_X_fr_link3::Type_fr_link4_X_fr_link3()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.2155;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link4_X_fr_link3& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link4_X_fr_link3::update(const JState& q) {
    Scalar s_q_jD_;
    Scalar c_q_jD_;
    
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jD_ = TRAIT::cos( q(JD));
    
    (*this)(0,0) =  c_q_jD_;
    (*this)(0,2) =  s_q_jD_;
    (*this)(0,4) = ( 0.2155 *  c_q_jD_);
    (*this)(1,0) = - s_q_jD_;
    (*this)(1,2) =  c_q_jD_;
    (*this)(1,4) = (- 0.2155 *  s_q_jD_);
    (*this)(3,3) =  c_q_jD_;
    (*this)(3,5) =  s_q_jD_;
    (*this)(4,3) = - s_q_jD_;
    (*this)(4,5) =  c_q_jD_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link3_X_fr_link4::Type_fr_link3_X_fr_link4()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0.2155;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link3_X_fr_link4& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link3_X_fr_link4::update(const JState& q) {
    Scalar s_q_jD_;
    Scalar c_q_jD_;
    
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jD_ = TRAIT::cos( q(JD));
    
    (*this)(0,0) =  c_q_jD_;
    (*this)(0,1) = - s_q_jD_;
    (*this)(1,3) = ( 0.2155 *  c_q_jD_);
    (*this)(1,4) = (- 0.2155 *  s_q_jD_);
    (*this)(2,0) =  s_q_jD_;
    (*this)(2,1) =  c_q_jD_;
    (*this)(3,3) =  c_q_jD_;
    (*this)(3,4) = - s_q_jD_;
    (*this)(5,3) =  s_q_jD_;
    (*this)(5,4) =  c_q_jD_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link5_X_fr_link4::Type_fr_link5_X_fr_link4()
{
    (*this)(0,1) = 0;
    (*this)(0,4) = 0;
    (*this)(1,1) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link5_X_fr_link4& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link5_X_fr_link4::update(const JState& q) {
    Scalar s_q_jE_;
    Scalar c_q_jE_;
    
    s_q_jE_ = TRAIT::sin( q(JE));
    c_q_jE_ = TRAIT::cos( q(JE));
    
    (*this)(0,0) = - c_q_jE_;
    (*this)(0,2) =  s_q_jE_;
    (*this)(0,3) = ( 0.1845 *  s_q_jE_);
    (*this)(0,5) = ( 0.1845 *  c_q_jE_);
    (*this)(1,0) =  s_q_jE_;
    (*this)(1,2) =  c_q_jE_;
    (*this)(1,3) = ( 0.1845 *  c_q_jE_);
    (*this)(1,5) = (- 0.1845 *  s_q_jE_);
    (*this)(3,3) = - c_q_jE_;
    (*this)(3,5) =  s_q_jE_;
    (*this)(4,3) =  s_q_jE_;
    (*this)(4,5) =  c_q_jE_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link4_X_fr_link5::Type_fr_link4_X_fr_link5()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link4_X_fr_link5& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link4_X_fr_link5::update(const JState& q) {
    Scalar s_q_jE_;
    Scalar c_q_jE_;
    
    s_q_jE_ = TRAIT::sin( q(JE));
    c_q_jE_ = TRAIT::cos( q(JE));
    
    (*this)(0,0) = - c_q_jE_;
    (*this)(0,1) =  s_q_jE_;
    (*this)(0,3) = ( 0.1845 *  s_q_jE_);
    (*this)(0,4) = ( 0.1845 *  c_q_jE_);
    (*this)(2,0) =  s_q_jE_;
    (*this)(2,1) =  c_q_jE_;
    (*this)(2,3) = ( 0.1845 *  c_q_jE_);
    (*this)(2,4) = (- 0.1845 *  s_q_jE_);
    (*this)(3,3) = - c_q_jE_;
    (*this)(3,4) =  s_q_jE_;
    (*this)(5,3) =  s_q_jE_;
    (*this)(5,4) =  c_q_jE_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link6_X_fr_link5::Type_fr_link6_X_fr_link5()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(0,5) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.2155;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 1.0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link6_X_fr_link5& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link6_X_fr_link5::update(const JState& q) {
    Scalar s_q_jF_;
    Scalar c_q_jF_;
    
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jF_ = TRAIT::cos( q(JF));
    
    (*this)(0,0) =  c_q_jF_;
    (*this)(0,2) =  s_q_jF_;
    (*this)(0,4) = ( 0.2155 *  c_q_jF_);
    (*this)(1,0) = - s_q_jF_;
    (*this)(1,2) =  c_q_jF_;
    (*this)(1,4) = (- 0.2155 *  s_q_jF_);
    (*this)(3,3) =  c_q_jF_;
    (*this)(3,5) =  s_q_jF_;
    (*this)(4,3) = - s_q_jF_;
    (*this)(4,5) =  c_q_jF_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link5_X_fr_link6::Type_fr_link5_X_fr_link6()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0.2155;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1.0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = - 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link5_X_fr_link6& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link5_X_fr_link6::update(const JState& q) {
    Scalar s_q_jF_;
    Scalar c_q_jF_;
    
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jF_ = TRAIT::cos( q(JF));
    
    (*this)(0,0) =  c_q_jF_;
    (*this)(0,1) = - s_q_jF_;
    (*this)(1,3) = ( 0.2155 *  c_q_jF_);
    (*this)(1,4) = (- 0.2155 *  s_q_jF_);
    (*this)(2,0) =  s_q_jF_;
    (*this)(2,1) =  c_q_jF_;
    (*this)(3,3) =  c_q_jF_;
    (*this)(3,4) = - s_q_jF_;
    (*this)(5,3) =  s_q_jF_;
    (*this)(5,4) =  c_q_jF_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link7_X_fr_link6::Type_fr_link7_X_fr_link6()
{
    (*this)(0,1) = 0;
    (*this)(0,4) = 0;
    (*this)(1,1) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,4) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,4) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 1;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link7_X_fr_link6& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link7_X_fr_link6::update(const JState& q) {
    Scalar s_q_jG_;
    Scalar c_q_jG_;
    
    s_q_jG_ = TRAIT::sin( q(JG));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = - c_q_jG_;
    (*this)(0,2) =  s_q_jG_;
    (*this)(0,3) = ( 0.081 *  s_q_jG_);
    (*this)(0,5) = ( 0.081 *  c_q_jG_);
    (*this)(1,0) =  s_q_jG_;
    (*this)(1,2) =  c_q_jG_;
    (*this)(1,3) = ( 0.081 *  c_q_jG_);
    (*this)(1,5) = (- 0.081 *  s_q_jG_);
    (*this)(3,3) = - c_q_jG_;
    (*this)(3,5) =  s_q_jG_;
    (*this)(4,3) =  s_q_jG_;
    (*this)(4,5) =  c_q_jG_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link6_X_fr_link7::Type_fr_link6_X_fr_link7()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,3) = 0;
    (*this)(4,4) = 0;
    (*this)(4,5) = 1;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = 0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link6_X_fr_link7& iit::Kuka::tpl::ForceTransforms<TRAIT>::Type_fr_link6_X_fr_link7::update(const JState& q) {
    Scalar s_q_jG_;
    Scalar c_q_jG_;
    
    s_q_jG_ = TRAIT::sin( q(JG));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = - c_q_jG_;
    (*this)(0,1) =  s_q_jG_;
    (*this)(0,3) = ( 0.081 *  s_q_jG_);
    (*this)(0,4) = ( 0.081 *  c_q_jG_);
    (*this)(2,0) =  s_q_jG_;
    (*this)(2,1) =  c_q_jG_;
    (*this)(2,3) = ( 0.081 *  c_q_jG_);
    (*this)(2,4) = (- 0.081 *  s_q_jG_);
    (*this)(3,3) = - c_q_jG_;
    (*this)(3,4) =  s_q_jG_;
    (*this)(5,3) =  s_q_jG_;
    (*this)(5,4) =  c_q_jG_;
    return *this;
}

template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link1::Type_fr_base0_X_fr_link1()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.1575;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link1& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link1::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar c_q_jA_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    c_q_jA_ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  c_q_jA_;
    (*this)(0,1) = - s_q_jA_;
    (*this)(1,0) =  s_q_jA_;
    (*this)(1,1) =  c_q_jA_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link2::Type_fr_base0_X_fr_link2()
{
    (*this)(0,3) = 0;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.36;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link2& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link2::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar s_q_jA_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jA_ = TRAIT::sin( q(JA));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = (- c_q_jA_ *  c_q_jB_);
    (*this)(0,1) = ( c_q_jA_ *  s_q_jB_);
    (*this)(0,2) = - s_q_jA_;
    (*this)(1,0) = (- s_q_jA_ *  c_q_jB_);
    (*this)(1,1) = ( s_q_jA_ *  s_q_jB_);
    (*this)(1,2) =  c_q_jA_;
    (*this)(2,0) =  s_q_jB_;
    (*this)(2,1) =  c_q_jB_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link3::Type_fr_base0_X_fr_link3()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link3& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link3::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jB_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    
    (*this)(0,0) = ((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_));
    (*this)(0,1) = (((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_));
    (*this)(0,2) = ( c_q_jA_ *  s_q_jB_);
    (*this)(0,3) = (( 0.2045 *  c_q_jA_) *  s_q_jB_);
    (*this)(1,0) = (( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_));
    (*this)(1,1) = (( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_));
    (*this)(1,2) = ( s_q_jA_ *  s_q_jB_);
    (*this)(1,3) = (( 0.2045 *  s_q_jA_) *  s_q_jB_);
    (*this)(2,0) = (- s_q_jB_ *  c_q_jC_);
    (*this)(2,1) = ( s_q_jB_ *  s_q_jC_);
    (*this)(2,2) =  c_q_jB_;
    (*this)(2,3) = (( 0.2045 *  c_q_jB_) +  0.36);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link4::Type_fr_base0_X_fr_link4()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link4& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link4::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jD_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jD_ = TRAIT::cos( q(JD));
    
    (*this)(0,0) = ((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(0,1) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(0,2) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(0,3) = (( 0.42 *  c_q_jA_) *  s_q_jB_);
    (*this)(1,0) = ((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(1,1) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,2) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(1,3) = (( 0.42 *  s_q_jA_) *  s_q_jB_);
    (*this)(2,0) = (( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_));
    (*this)(2,1) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(2,2) = (- s_q_jB_ *  s_q_jC_);
    (*this)(2,3) = (( 0.42 *  c_q_jB_) +  0.36);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link5::Type_fr_base0_X_fr_link5()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link5& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link5::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jE_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar c_q_jC_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jD_;
    Scalar c_q_jE_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jE_ = TRAIT::cos( q(JE));
    
    (*this)(0,0) = (((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,1) = ((((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(0,2) = ((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(0,3) = (((((( 0.1845 *  s_q_jA_) *  s_q_jC_) - ((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.1845 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) + (( 0.42 *  c_q_jA_) *  s_q_jB_));
    (*this)(1,0) = ((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(1,1) = (((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  c_q_jE_));
    (*this)(1,2) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,3) = (((((((- 0.1845 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.1845 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.1845 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) + (( 0.42 *  s_q_jA_) *  s_q_jB_));
    (*this)(2,0) = ((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_));
    (*this)(2,1) = (((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(2,2) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(2,3) = (((((( 0.1845 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.1845 *  c_q_jB_) *  c_q_jD_)) + ( 0.42 *  c_q_jB_)) +  0.36);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link6::Type_fr_base0_X_fr_link6()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link6& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link6::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jD_;
    Scalar s_q_jF_;
    Scalar s_q_jE_;
    Scalar c_q_jA_;
    Scalar c_q_jD_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jF_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jF_ = TRAIT::sin( q(JF));
    s_q_jE_ = TRAIT::sin( q(JE));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jF_ = TRAIT::cos( q(JF));
    
    (*this)(0,0) = ((((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,1) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(0,2) = (((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(0,3) = (((((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) + (( 0.42 *  c_q_jA_) *  s_q_jB_));
    (*this)(1,0) = ((((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(1,1) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,2) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(1,3) = (((((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) + (( 0.42 *  s_q_jA_) *  s_q_jB_));
    (*this)(2,0) = (((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + (((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(2,1) = (((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + ((( c_q_jB_ *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,2) = ((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(2,3) = (((((( 0.4 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.4 *  c_q_jB_) *  c_q_jD_)) + ( 0.42 *  c_q_jB_)) +  0.36);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link7::Type_fr_base0_X_fr_link7()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link7& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_link7::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jE_;
    Scalar s_q_jG_;
    Scalar s_q_jF_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jD_;
    Scalar c_q_jF_;
    Scalar c_q_jG_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jG_ = TRAIT::sin( q(JG));
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jF_ = TRAIT::cos( q(JF));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,1) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(0,2) = (((((((( 1.0 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 1.0 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 1.0 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + (((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(0,3) = ((((((((((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + (((((- 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.081 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + (((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) + ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) + (( 0.42 *  c_q_jA_) *  s_q_jB_));
    (*this)(1,0) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,1) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,2) = (((((((( 1.0 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 1.0 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 1.0 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,3) = ((((((((((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + (((( 0.081 *  c_q_jA_) *  c_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.081 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((- 0.081 *  c_q_jA_) *  s_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_)) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_)) + ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) + (( 0.42 *  s_q_jA_) *  s_q_jB_));
    (*this)(2,0) = ((((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((( s_q_jB_ *  s_q_jC_) *  s_q_jE_) + ((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(2,1) = ((((((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + (((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(2,2) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,3) = ((((((((((( 0.081 *  c_q_jB_) *  s_q_jD_) - ((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + ((( 0.081 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 0.081 *  c_q_jB_) *  c_q_jD_) + ((( 0.081 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_)) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_)) + (( 0.4 *  c_q_jB_) *  c_q_jD_)) + ( 0.42 *  c_q_jB_)) +  0.36);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_ee::Type_fr_base0_X_fr_ee()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_ee& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_ee::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jE_;
    Scalar s_q_jG_;
    Scalar s_q_jF_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jD_;
    Scalar c_q_jF_;
    Scalar c_q_jG_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jG_ = TRAIT::sin( q(JG));
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jF_ = TRAIT::cos( q(JF));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,1) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(0,2) = (((((((( 1.0 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 1.0 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 1.0 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + (((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(0,3) = ((((((((((( 0.241 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.241 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + (((((- 0.241 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.241 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.241 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + (((( 0.241 *  s_q_jA_) *  s_q_jC_) - ((( 0.241 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) + ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) + (( 0.42 *  c_q_jA_) *  s_q_jB_));
    (*this)(1,0) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,1) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,2) = (((((((( 1.0 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 1.0 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 1.0 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,3) = ((((((((((( 0.241 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.241 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + (((( 0.241 *  c_q_jA_) *  c_q_jC_) - ((( 0.241 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.241 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((- 0.241 *  c_q_jA_) *  s_q_jC_) - ((( 0.241 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_)) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_)) + ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) + (( 0.42 *  s_q_jA_) *  s_q_jB_));
    (*this)(2,0) = ((((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((( s_q_jB_ *  s_q_jC_) *  s_q_jE_) + ((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(2,1) = ((((((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + (((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + (((( s_q_jB_ *  s_q_jC_) *  c_q_jE_) + (((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(2,2) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,3) = ((((((((((( 0.241 *  c_q_jB_) *  s_q_jD_) - ((( 0.241 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + ((( 0.241 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 0.241 *  c_q_jB_) *  c_q_jD_) + ((( 0.241 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_)) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_)) + (( 0.4 *  c_q_jB_) *  c_q_jD_)) + ( 0.42 *  c_q_jB_)) +  0.36);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link1_X_fr_base0::Type_fr_link1_X_fr_base0()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = - 0.1575;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link1_X_fr_base0& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link1_X_fr_base0::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar c_q_jA_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    c_q_jA_ = TRAIT::cos( q(JA));
    
    (*this)(0,0) =  c_q_jA_;
    (*this)(0,1) =  s_q_jA_;
    (*this)(1,0) = - s_q_jA_;
    (*this)(1,1) =  c_q_jA_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link2_X_fr_base0::Type_fr_link2_X_fr_base0()
{
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link2_X_fr_base0& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link2_X_fr_base0::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = (- c_q_jA_ *  c_q_jB_);
    (*this)(0,1) = (- s_q_jA_ *  c_q_jB_);
    (*this)(0,2) =  s_q_jB_;
    (*this)(0,3) = (- 0.36 *  s_q_jB_);
    (*this)(1,0) = ( c_q_jA_ *  s_q_jB_);
    (*this)(1,1) = ( s_q_jA_ *  s_q_jB_);
    (*this)(1,2) =  c_q_jB_;
    (*this)(1,3) = (- 0.36 *  c_q_jB_);
    (*this)(2,0) = - s_q_jA_;
    (*this)(2,1) =  c_q_jA_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link3_X_fr_base0::Type_fr_link3_X_fr_base0()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link3_X_fr_base0& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link3_X_fr_base0::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jB_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    
    (*this)(0,0) = ((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_));
    (*this)(0,1) = (( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_));
    (*this)(0,2) = (- s_q_jB_ *  c_q_jC_);
    (*this)(0,3) = (( 0.36 *  s_q_jB_) *  c_q_jC_);
    (*this)(1,0) = (((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_));
    (*this)(1,1) = (( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_));
    (*this)(1,2) = ( s_q_jB_ *  s_q_jC_);
    (*this)(1,3) = ((- 0.36 *  s_q_jB_) *  s_q_jC_);
    (*this)(2,0) = ( c_q_jA_ *  s_q_jB_);
    (*this)(2,1) = ( s_q_jA_ *  s_q_jB_);
    (*this)(2,2) =  c_q_jB_;
    (*this)(2,3) = ((- 0.36 *  c_q_jB_) -  0.2045);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link4_X_fr_base0::Type_fr_link4_X_fr_base0()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link4_X_fr_base0& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link4_X_fr_base0::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jD_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jD_ = TRAIT::cos( q(JD));
    
    (*this)(0,0) = ((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(0,1) = ((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_));
    (*this)(0,2) = (( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_));
    (*this)(0,3) = (((- 0.42 - ( 0.36 *  c_q_jB_)) *  s_q_jD_) + ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_));
    (*this)(1,0) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,1) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,2) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(1,3) = (((- 0.42 - ( 0.36 *  c_q_jB_)) *  c_q_jD_) - ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_));
    (*this)(2,0) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(2,1) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(2,2) = (- s_q_jB_ *  s_q_jC_);
    (*this)(2,3) = (( 0.36 *  s_q_jB_) *  s_q_jC_);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link5_X_fr_base0::Type_fr_link5_X_fr_base0()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link5_X_fr_base0& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link5_X_fr_base0::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jE_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar c_q_jC_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jD_;
    Scalar c_q_jE_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jE_ = TRAIT::cos( q(JE));
    
    (*this)(0,0) = (((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,1) = ((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,2) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_));
    (*this)(0,3) = (((( 0.36 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_) + ((((( 0.36 *  c_q_jB_) +  0.42) *  s_q_jD_) - ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_));
    (*this)(1,0) = ((((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(1,1) = (((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  c_q_jE_));
    (*this)(1,2) = ((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(1,3) = (((((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) + (((- 0.36 *  c_q_jB_) -  0.42) *  s_q_jD_)) *  s_q_jE_) + ((( 0.36 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_));
    (*this)(2,0) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(2,1) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(2,2) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(2,3) = (((((- 0.36 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + ((- 0.42 - ( 0.36 *  c_q_jB_)) *  c_q_jD_)) -  0.1845);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link6_X_fr_base0::Type_fr_link6_X_fr_base0()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link6_X_fr_base0& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link6_X_fr_base0::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jD_;
    Scalar s_q_jF_;
    Scalar s_q_jE_;
    Scalar c_q_jA_;
    Scalar c_q_jD_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jF_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jF_ = TRAIT::sin( q(JF));
    s_q_jE_ = TRAIT::sin( q(JE));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jF_ = TRAIT::cos( q(JF));
    
    (*this)(0,0) = ((((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,1) = ((((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  s_q_jF_) + ((((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_) + (((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,2) = ((((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + ((((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,3) = ((((- 0.4 + (((- 0.36 *  c_q_jB_) -  0.42) *  c_q_jD_)) - ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + (((((( 0.42 + ( 0.36 *  c_q_jB_)) *  s_q_jD_) - ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + ((( 0.36 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(1,0) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,1) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,2) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,3) = (((((((- 0.42 - ( 0.36 *  c_q_jB_)) *  s_q_jD_) + ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) - ((( 0.36 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((- 0.4 + (((- 0.36 *  c_q_jB_) -  0.42) *  c_q_jD_)) - ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,0) = (((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(2,1) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(2,2) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(2,3) = (((((( 0.36 *  c_q_jB_) +  0.42) *  s_q_jD_) - ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_) - ((( 0.36 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_));
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link7_X_fr_base0::Type_fr_link7_X_fr_base0()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link7_X_fr_base0& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link7_X_fr_base0::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jE_;
    Scalar s_q_jG_;
    Scalar s_q_jF_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jD_;
    Scalar c_q_jF_;
    Scalar c_q_jG_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jG_ = TRAIT::sin( q(JG));
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jF_ = TRAIT::cos( q(JF));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,1) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,2) = (((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_) + (((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,3) = ((((((( 0.42 + ( 0.36 *  c_q_jB_)) *  s_q_jD_) - ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_) - ((( 0.36 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_)) *  s_q_jG_) + ((((((((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) + (((- 0.36 *  c_q_jB_) -  0.42) *  s_q_jD_)) *  c_q_jE_) - ((( 0.36 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + (((((( 0.36 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.42 + ( 0.36 *  c_q_jB_)) *  c_q_jD_)) +  0.4) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,0) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,1) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,2) = (((((((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,3) = (((((((( 0.36 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_) + ((((( 0.36 *  c_q_jB_) +  0.42) *  s_q_jD_) - ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- 0.36 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + ((- 0.42 - ( 0.36 *  c_q_jB_)) *  c_q_jD_)) -  0.4) *  s_q_jF_)) *  s_q_jG_) + (((((( 0.42 + ( 0.36 *  c_q_jB_)) *  s_q_jD_) - ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_) - ((( 0.36 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_)) *  c_q_jG_));
    (*this)(2,0) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,1) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,2) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,3) = ((((((((- 0.42 - ( 0.36 *  c_q_jB_)) *  s_q_jD_) + ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) - ((( 0.36 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((- 0.4 + (((- 0.36 *  c_q_jB_) -  0.42) *  c_q_jD_)) - ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_)) -  0.081);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ee_X_fr_base0::Type_fr_ee_X_fr_base0()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ee_X_fr_base0& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_ee_X_fr_base0::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar s_q_jE_;
    Scalar s_q_jG_;
    Scalar s_q_jF_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jD_;
    Scalar c_q_jF_;
    Scalar c_q_jG_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jG_ = TRAIT::sin( q(JG));
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jF_ = TRAIT::cos( q(JF));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = (((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + (((((((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,1) = (((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + (((((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,2) = (((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  s_q_jG_) + ((((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_) + (((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((- s_q_jB_ *  c_q_jC_) *  s_q_jD_) - ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  c_q_jG_));
    (*this)(0,3) = ((((((( 0.42 + ( 0.36 *  c_q_jB_)) *  s_q_jD_) - ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_) - ((( 0.36 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_)) *  s_q_jG_) + ((((((((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) + (((- 0.36 *  c_q_jB_) -  0.42) *  s_q_jD_)) *  c_q_jE_) - ((( 0.36 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + (((((( 0.36 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.42 + ( 0.36 *  c_q_jB_)) *  c_q_jD_)) +  0.4) *  s_q_jF_)) *  c_q_jG_));
    (*this)(1,0) = (((((((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  c_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,1) = ((((((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_) + ((((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,2) = (((((((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_)) *  s_q_jF_)) *  s_q_jG_) + ((((( 1.0 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_) + ((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_)) *  c_q_jG_));
    (*this)(1,3) = (((((((( 0.36 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_) + ((((( 0.36 *  c_q_jB_) +  0.42) *  s_q_jD_) - ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_)) *  c_q_jF_) + ((((((- 0.36 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + ((- 0.42 - ( 0.36 *  c_q_jB_)) *  c_q_jD_)) -  0.4) *  s_q_jF_)) *  s_q_jG_) + (((((( 0.42 + ( 0.36 *  c_q_jB_)) *  s_q_jD_) - ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  s_q_jE_) - ((( 0.36 *  s_q_jB_) *  s_q_jC_) *  c_q_jE_)) *  c_q_jG_));
    (*this)(2,0) = ((((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( c_q_jA_ *  s_q_jB_) *  c_q_jD_) + ((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,1) = ((((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( s_q_jA_ *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,2) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,3) = ((((((((- 0.42 - ( 0.36 *  c_q_jB_)) *  s_q_jD_) + ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) - ((( 0.36 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((- 0.4 + (((- 0.36 *  c_q_jB_) -  0.42) *  c_q_jD_)) - ((( 0.36 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_)) -  0.241);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jA::Type_fr_base0_X_fr_jA()
{
    (*this)(0,0) = 1;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 1;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1;
    (*this)(2,3) = 0.1575;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jA& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jA::update(const JState& q) {
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jB::Type_fr_base0_X_fr_jB()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.36;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jB& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jB::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar c_q_jA_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    c_q_jA_ = TRAIT::cos( q(JA));
    
    (*this)(0,0) = - c_q_jA_;
    (*this)(0,2) = - s_q_jA_;
    (*this)(1,0) = - s_q_jA_;
    (*this)(1,2) =  c_q_jA_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jC::Type_fr_base0_X_fr_jC()
{
    (*this)(2,1) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jC& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jC::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jB_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = ( c_q_jA_ *  c_q_jB_);
    (*this)(0,1) = - s_q_jA_;
    (*this)(0,2) = ( c_q_jA_ *  s_q_jB_);
    (*this)(0,3) = (( 0.2045 *  c_q_jA_) *  s_q_jB_);
    (*this)(1,0) = ( s_q_jA_ *  c_q_jB_);
    (*this)(1,1) =  c_q_jA_;
    (*this)(1,2) = ( s_q_jA_ *  s_q_jB_);
    (*this)(1,3) = (( 0.2045 *  s_q_jA_) *  s_q_jB_);
    (*this)(2,0) = - s_q_jB_;
    (*this)(2,2) =  c_q_jB_;
    (*this)(2,3) = (( 0.2045 *  c_q_jB_) +  0.36);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jD::Type_fr_base0_X_fr_jD()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jD& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jD::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jB_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    
    (*this)(0,0) = ((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_));
    (*this)(0,1) = ( c_q_jA_ *  s_q_jB_);
    (*this)(0,2) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(0,3) = (( 0.42 *  c_q_jA_) *  s_q_jB_);
    (*this)(1,0) = (( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_));
    (*this)(1,1) = ( s_q_jA_ *  s_q_jB_);
    (*this)(1,2) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(1,3) = (( 0.42 *  s_q_jA_) *  s_q_jB_);
    (*this)(2,0) = (- s_q_jB_ *  c_q_jC_);
    (*this)(2,1) =  c_q_jB_;
    (*this)(2,2) = (- s_q_jB_ *  s_q_jC_);
    (*this)(2,3) = (( 0.42 *  c_q_jB_) +  0.36);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jE::Type_fr_base0_X_fr_jE()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jE& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jE::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jD_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jD_ = TRAIT::cos( q(JD));
    
    (*this)(0,0) = (((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_));
    (*this)(0,1) = ((( c_q_jA_ *  c_q_jB_) *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(0,2) = ((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(0,3) = (((((( 0.1845 *  s_q_jA_) *  s_q_jC_) - ((( 0.1845 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.1845 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) + (( 0.42 *  c_q_jA_) *  s_q_jB_));
    (*this)(1,0) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_));
    (*this)(1,1) = ((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(1,2) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,3) = (((((((- 0.1845 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.1845 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.1845 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) + (( 0.42 *  s_q_jA_) *  s_q_jB_));
    (*this)(2,0) = ((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_));
    (*this)(2,1) = (- s_q_jB_ *  s_q_jC_);
    (*this)(2,2) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(2,3) = (((((( 0.1845 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.1845 *  c_q_jB_) *  c_q_jD_)) + ( 0.42 *  c_q_jB_)) +  0.36);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jF::Type_fr_base0_X_fr_jF()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jF& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jF::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jE_;
    Scalar s_q_jB_;
    Scalar s_q_jD_;
    Scalar c_q_jC_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jD_;
    Scalar c_q_jE_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jE_ = TRAIT::sin( q(JE));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jE_ = TRAIT::cos( q(JE));
    
    (*this)(0,0) = (((( s_q_jA_ *  c_q_jC_) + (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_) + ((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(0,1) = ((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + (( c_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(0,2) = (((((( s_q_jA_ *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(0,3) = (((((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) + ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) + (( 0.42 *  c_q_jA_) *  s_q_jB_));
    (*this)(1,0) = ((((( s_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + (((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  c_q_jE_));
    (*this)(1,1) = (((((- s_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jD_) + (( s_q_jA_ *  s_q_jB_) *  c_q_jD_));
    (*this)(1,2) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(1,3) = (((((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_) + ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) + (( 0.42 *  s_q_jA_) *  s_q_jB_));
    (*this)(2,0) = ((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  c_q_jE_) - (( s_q_jB_ *  s_q_jC_) *  s_q_jE_));
    (*this)(2,1) = ((( s_q_jB_ *  c_q_jC_) *  s_q_jD_) + ( c_q_jB_ *  c_q_jD_));
    (*this)(2,2) = ((((( s_q_jB_ *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(2,3) = (((((( 0.4 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_) + (( 0.4 *  c_q_jB_) *  c_q_jD_)) + ( 0.42 *  c_q_jB_)) +  0.36);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jG::Type_fr_base0_X_fr_jG()
{
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jG& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base0_X_fr_jG::update(const JState& q) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jD_;
    Scalar s_q_jB_;
    Scalar s_q_jF_;
    Scalar s_q_jE_;
    Scalar c_q_jA_;
    Scalar c_q_jB_;
    Scalar c_q_jC_;
    Scalar c_q_jD_;
    Scalar c_q_jE_;
    Scalar c_q_jF_;
    
    s_q_jA_ = TRAIT::sin( q(JA));
    s_q_jC_ = TRAIT::sin( q(JC));
    s_q_jD_ = TRAIT::sin( q(JD));
    s_q_jB_ = TRAIT::sin( q(JB));
    s_q_jF_ = TRAIT::sin( q(JF));
    s_q_jE_ = TRAIT::sin( q(JE));
    c_q_jA_ = TRAIT::cos( q(JA));
    c_q_jB_ = TRAIT::cos( q(JB));
    c_q_jC_ = TRAIT::cos( q(JC));
    c_q_jD_ = TRAIT::cos( q(JD));
    c_q_jE_ = TRAIT::cos( q(JE));
    c_q_jF_ = TRAIT::cos( q(JF));
    
    (*this)(0,0) = ((((((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + (((((( c_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( c_q_jA_ *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(0,1) = ((((((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( c_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(0,2) = (((((((( 1.0 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 1.0 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((((- c_q_jA_ *  c_q_jB_) *  s_q_jC_) - ( s_q_jA_ *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 1.0 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + (((( 1.0 *  s_q_jA_) *  s_q_jC_) - (( c_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(0,3) = ((((((((((( 0.081 *  c_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.081 *  s_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + (((((- 0.081 *  c_q_jA_) *  c_q_jB_) *  s_q_jC_) - (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.081 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_) + (((( 0.081 *  s_q_jA_) *  s_q_jC_) - ((( 0.081 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_)) + (((( 0.4 *  s_q_jA_) *  s_q_jC_) - ((( 0.4 *  c_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) + ((( 0.4 *  c_q_jA_) *  s_q_jB_) *  c_q_jD_)) + (( 0.42 *  c_q_jA_) *  s_q_jB_));
    (*this)(1,0) = (((((( c_q_jA_ *  s_q_jC_) + (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  c_q_jD_)) *  s_q_jF_) + (((((( s_q_jA_ *  s_q_jB_) *  s_q_jD_) + (((( s_q_jA_ *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + ((( c_q_jA_ *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  c_q_jF_));
    (*this)(1,1) = ((((((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  c_q_jD_) - (( s_q_jA_ *  s_q_jB_) *  s_q_jD_)) *  s_q_jE_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  c_q_jE_));
    (*this)(1,2) = (((((((( 1.0 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 1.0 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 1.0 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_) + (((- c_q_jA_ *  s_q_jC_) - (( s_q_jA_ *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_));
    (*this)(1,3) = ((((((((((( 0.081 *  s_q_jA_) *  s_q_jB_) *  s_q_jD_) + ((((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) + (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  c_q_jD_)) *  c_q_jE_) + (((( 0.081 *  c_q_jA_) *  c_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + ((((( 0.081 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_) + ((((- 0.081 *  c_q_jA_) *  s_q_jC_) - ((( 0.081 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_)) *  s_q_jD_)) *  c_q_jF_)) + (((((- 0.4 *  s_q_jA_) *  c_q_jB_) *  c_q_jC_) - (( 0.4 *  c_q_jA_) *  s_q_jC_)) *  s_q_jD_)) + ((( 0.4 *  s_q_jA_) *  s_q_jB_) *  c_q_jD_)) + (( 0.42 *  s_q_jA_) *  s_q_jB_));
    (*this)(2,0) = ((((- c_q_jB_ *  c_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  s_q_jF_) + ((((( c_q_jB_ *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  c_q_jF_));
    (*this)(2,1) = (((((( 1.0 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_) - ( c_q_jB_ *  s_q_jD_)) *  s_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  c_q_jE_));
    (*this)(2,2) = ((((((( 1.0 *  c_q_jB_) *  s_q_jD_) - (( s_q_jB_ *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + (( s_q_jB_ *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jB_) *  c_q_jD_) + (( s_q_jB_ *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_));
    (*this)(2,3) = ((((((((((( 0.081 *  c_q_jB_) *  s_q_jD_) - ((( 0.081 *  s_q_jB_) *  c_q_jC_) *  c_q_jD_)) *  c_q_jE_) + ((( 0.081 *  s_q_jB_) *  s_q_jC_) *  s_q_jE_)) *  s_q_jF_) + (((( 0.081 *  c_q_jB_) *  c_q_jD_) + ((( 0.081 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_)) *  c_q_jF_)) + ((( 0.4 *  s_q_jB_) *  c_q_jC_) *  s_q_jD_)) + (( 0.4 *  c_q_jB_) *  c_q_jD_)) + ( 0.42 *  c_q_jB_)) +  0.36);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link2_X_fr_link1::Type_fr_link2_X_fr_link1()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link2_X_fr_link1& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link2_X_fr_link1::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar c_q_jB_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jB_ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = - c_q_jB_;
    (*this)(0,2) =  s_q_jB_;
    (*this)(0,3) = (- 0.2025 *  s_q_jB_);
    (*this)(1,0) =  s_q_jB_;
    (*this)(1,2) =  c_q_jB_;
    (*this)(1,3) = (- 0.2025 *  c_q_jB_);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link1_X_fr_link2::Type_fr_link1_X_fr_link2()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.2025;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link1_X_fr_link2& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link1_X_fr_link2::update(const JState& q) {
    Scalar s_q_jB_;
    Scalar c_q_jB_;
    
    s_q_jB_ = TRAIT::sin( q(JB));
    c_q_jB_ = TRAIT::cos( q(JB));
    
    (*this)(0,0) = - c_q_jB_;
    (*this)(0,1) =  s_q_jB_;
    (*this)(2,0) =  s_q_jB_;
    (*this)(2,1) =  c_q_jB_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link3_X_fr_link2::Type_fr_link3_X_fr_link2()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.2045;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link3_X_fr_link2& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link3_X_fr_link2::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar c_q_jC_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    c_q_jC_ = TRAIT::cos( q(JC));
    
    (*this)(0,0) = - c_q_jC_;
    (*this)(0,2) =  s_q_jC_;
    (*this)(1,0) =  s_q_jC_;
    (*this)(1,2) =  c_q_jC_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link2_X_fr_link3::Type_fr_link2_X_fr_link3()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.2045;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link2_X_fr_link3& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link2_X_fr_link3::update(const JState& q) {
    Scalar s_q_jC_;
    Scalar c_q_jC_;
    
    s_q_jC_ = TRAIT::sin( q(JC));
    c_q_jC_ = TRAIT::cos( q(JC));
    
    (*this)(0,0) = - c_q_jC_;
    (*this)(0,1) =  s_q_jC_;
    (*this)(2,0) =  s_q_jC_;
    (*this)(2,1) =  c_q_jC_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link4_X_fr_link3::Type_fr_link4_X_fr_link3()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link4_X_fr_link3& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link4_X_fr_link3::update(const JState& q) {
    Scalar s_q_jD_;
    Scalar c_q_jD_;
    
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jD_ = TRAIT::cos( q(JD));
    
    (*this)(0,0) =  c_q_jD_;
    (*this)(0,2) =  s_q_jD_;
    (*this)(0,3) = (- 0.2155 *  s_q_jD_);
    (*this)(1,0) = - s_q_jD_;
    (*this)(1,2) =  c_q_jD_;
    (*this)(1,3) = (- 0.2155 *  c_q_jD_);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link3_X_fr_link4::Type_fr_link3_X_fr_link4()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.2155;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link3_X_fr_link4& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link3_X_fr_link4::update(const JState& q) {
    Scalar s_q_jD_;
    Scalar c_q_jD_;
    
    s_q_jD_ = TRAIT::sin( q(JD));
    c_q_jD_ = TRAIT::cos( q(JD));
    
    (*this)(0,0) =  c_q_jD_;
    (*this)(0,1) = - s_q_jD_;
    (*this)(2,0) =  s_q_jD_;
    (*this)(2,1) =  c_q_jD_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link5_X_fr_link4::Type_fr_link5_X_fr_link4()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.1845;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link5_X_fr_link4& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link5_X_fr_link4::update(const JState& q) {
    Scalar s_q_jE_;
    Scalar c_q_jE_;
    
    s_q_jE_ = TRAIT::sin( q(JE));
    c_q_jE_ = TRAIT::cos( q(JE));
    
    (*this)(0,0) = - c_q_jE_;
    (*this)(0,2) =  s_q_jE_;
    (*this)(1,0) =  s_q_jE_;
    (*this)(1,2) =  c_q_jE_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link4_X_fr_link5::Type_fr_link4_X_fr_link5()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.1845;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link4_X_fr_link5& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link4_X_fr_link5::update(const JState& q) {
    Scalar s_q_jE_;
    Scalar c_q_jE_;
    
    s_q_jE_ = TRAIT::sin( q(JE));
    c_q_jE_ = TRAIT::cos( q(JE));
    
    (*this)(0,0) = - c_q_jE_;
    (*this)(0,1) =  s_q_jE_;
    (*this)(2,0) =  s_q_jE_;
    (*this)(2,1) =  c_q_jE_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link6_X_fr_link5::Type_fr_link6_X_fr_link5()
{
    (*this)(0,1) = 0;
    (*this)(1,1) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link6_X_fr_link5& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link6_X_fr_link5::update(const JState& q) {
    Scalar s_q_jF_;
    Scalar c_q_jF_;
    
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jF_ = TRAIT::cos( q(JF));
    
    (*this)(0,0) =  c_q_jF_;
    (*this)(0,2) =  s_q_jF_;
    (*this)(0,3) = (- 0.2155 *  s_q_jF_);
    (*this)(1,0) = - s_q_jF_;
    (*this)(1,2) =  c_q_jF_;
    (*this)(1,3) = (- 0.2155 *  c_q_jF_);
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link5_X_fr_link6::Type_fr_link5_X_fr_link6()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = - 1;
    (*this)(1,3) = 0;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0.2155;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link5_X_fr_link6& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link5_X_fr_link6::update(const JState& q) {
    Scalar s_q_jF_;
    Scalar c_q_jF_;
    
    s_q_jF_ = TRAIT::sin( q(JF));
    c_q_jF_ = TRAIT::cos( q(JF));
    
    (*this)(0,0) =  c_q_jF_;
    (*this)(0,1) = - s_q_jF_;
    (*this)(2,0) =  s_q_jF_;
    (*this)(2,1) =  c_q_jF_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link7_X_fr_link6::Type_fr_link7_X_fr_link6()
{
    (*this)(0,1) = 0;
    (*this)(0,3) = 0;
    (*this)(1,1) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 1;
    (*this)(2,2) = 0;
    (*this)(2,3) = - 0.081;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link7_X_fr_link6& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link7_X_fr_link6::update(const JState& q) {
    Scalar s_q_jG_;
    Scalar c_q_jG_;
    
    s_q_jG_ = TRAIT::sin( q(JG));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = - c_q_jG_;
    (*this)(0,2) =  s_q_jG_;
    (*this)(1,0) =  s_q_jG_;
    (*this)(1,2) =  c_q_jG_;
    return *this;
}
template <typename TRAIT>
iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link6_X_fr_link7::Type_fr_link6_X_fr_link7()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.081;
    (*this)(2,2) = 0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link6_X_fr_link7& iit::Kuka::tpl::HomogeneousTransforms<TRAIT>::Type_fr_link6_X_fr_link7::update(const JState& q) {
    Scalar s_q_jG_;
    Scalar c_q_jG_;
    
    s_q_jG_ = TRAIT::sin( q(JG));
    c_q_jG_ = TRAIT::cos( q(JG));
    
    (*this)(0,0) = - c_q_jG_;
    (*this)(0,1) =  s_q_jG_;
    (*this)(2,0) =  s_q_jG_;
    (*this)(2,1) =  c_q_jG_;
    return *this;
}

