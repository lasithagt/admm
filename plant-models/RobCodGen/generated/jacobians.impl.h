
template <typename TRAIT>
iit::Kuka::tpl::Jacobians<TRAIT>::Jacobians
    ()
     : 
    fr_base0_J_fr_ee()
{
    updateParameters();
}

template <typename TRAIT>
void iit::Kuka::tpl::Jacobians<TRAIT>::updateParameters() {
}


template <typename TRAIT>
iit::Kuka::tpl::Jacobians<TRAIT>::Type_fr_base0_J_fr_ee::Type_fr_base0_J_fr_ee()
{
    (*this)(0,0) = 0;
    (*this)(0,1) = 0;
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,0) = 0;
    (*this)(1,1) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 1.0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 1.0;
    (*this)(2,5) = 0;
    (*this)(3,6) = 0;
    (*this)(4,6) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,6) = 0;
}

template <typename TRAIT>
const typename iit::Kuka::tpl::Jacobians<TRAIT>::Type_fr_base0_J_fr_ee& iit::Kuka::tpl::Jacobians<TRAIT>::Type_fr_base0_J_fr_ee::update(const JState& jState) {
    Scalar s_q_jA_;
    Scalar s_q_jC_;
    Scalar s_q_jE_;
    Scalar s_q_jF_;
    Scalar c_q_jA_;
    Scalar c_q_jC_;
    Scalar c_q_jE_;
    Scalar c_q_jF_;
    
    s_q_jA_ = TRAIT::sin( jState(JA));
    s_q_jC_ = TRAIT::sin( jState(JC));
    s_q_jE_ = TRAIT::sin( jState(JE));
    s_q_jF_ = TRAIT::sin( jState(JF));
    c_q_jA_ = TRAIT::cos( jState(JA));
    c_q_jC_ = TRAIT::cos( jState(JC));
    c_q_jE_ = TRAIT::cos( jState(JE));
    c_q_jF_ = TRAIT::cos( jState(JF));
    
    (*this)(0,5) = (((( s_q_jA_ *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  s_q_jE_) + ((( s_q_jA_ *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  c_q_jE_));
    (*this)(0,6) = (((((- s_q_jA_ *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jE_) + ((( s_q_jA_ *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  c_q_jE_)) *  s_q_jF_);
    (*this)(1,5) = ((((- s_q_jA_ *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  s_q_jE_) + ((( s_q_jA_ *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_)) *  c_q_jE_));
    (*this)(1,6) = ((((( c_q_jA_ *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  s_q_jE_) + (((- s_q_jA_ *  c_q_jC_) - ( c_q_jA_ *  s_q_jC_)) *  c_q_jE_)) *  s_q_jF_);
    (*this)(2,6) =  c_q_jF_;
    (*this)(3,0) = ((((((((- 0.081 *  c_q_jA_) *  s_q_jC_) - (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((( 0.081 *  c_q_jA_) *  c_q_jC_) - (( 0.081 *  s_q_jA_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  jState(JD))) + ( c_q_jA_ *  jState(JB)));
    (*this)(3,1) =  s_q_jA_;
    (*this)(3,2) = (((((((- 0.081 *  c_q_jA_) *  s_q_jC_) - (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  c_q_jE_) + (((( 0.081 *  c_q_jA_) *  c_q_jC_) - (( 0.081 *  s_q_jA_) *  s_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  c_q_jA_) *  c_q_jC_) - ( s_q_jA_ *  s_q_jC_)) *  jState(JD)));
    (*this)(3,3) = (( c_q_jA_ *  s_q_jC_) + ( s_q_jA_ *  c_q_jC_));
    (*this)(3,4) = (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + (((( 0.081 *  s_q_jA_) *  c_q_jC_) + (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  c_q_jE_)) *  s_q_jF_);
    (*this)(3,5) = (((((( 0.081 *  s_q_jA_) *  c_q_jC_) + (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  s_q_jE_) + (((( 0.081 *  c_q_jA_) *  c_q_jC_) - (( 0.081 *  s_q_jA_) *  s_q_jC_)) *  c_q_jE_)) *  c_q_jF_);
    (*this)(4,0) = (((((((( 0.081 *  c_q_jA_) *  c_q_jC_) - (( 0.081 *  s_q_jA_) *  s_q_jC_)) *  c_q_jE_) + (((( 0.081 *  c_q_jA_) *  s_q_jC_) + (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  s_q_jA_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  jState(JD))) + ( s_q_jA_ *  jState(JB)));
    (*this)(4,1) = - c_q_jA_;
    (*this)(4,2) = ((((((( 0.081 *  c_q_jA_) *  c_q_jC_) - (( 0.081 *  s_q_jA_) *  s_q_jC_)) *  c_q_jE_) + (((( 0.081 *  c_q_jA_) *  s_q_jC_) + (( 0.081 *  s_q_jA_) *  c_q_jC_)) *  s_q_jE_)) *  s_q_jF_) + (((( 1.0 *  s_q_jA_) *  c_q_jC_) + ( c_q_jA_ *  s_q_jC_)) *  jState(JD)));
    (*this)(4,3) = (( s_q_jA_ *  s_q_jC_) - ( c_q_jA_ *  c_q_jC_));
    (*this)(4,4) = ((((((- 0.081 *  s_q_jA_) *  c_q_jC_) - (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  s_q_jE_) + (((( 0.081 *  s_q_jA_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  c_q_jE_)) *  s_q_jF_);
    (*this)(4,5) = (((((( 0.081 *  s_q_jA_) *  s_q_jC_) - (( 0.081 *  c_q_jA_) *  c_q_jC_)) *  s_q_jE_) + (((( 0.081 *  s_q_jA_) *  c_q_jC_) + (( 0.081 *  c_q_jA_) *  s_q_jC_)) *  c_q_jE_)) *  c_q_jF_);
    (*this)(5,5) = ( 0.081 *  s_q_jF_);
    return *this;
}
