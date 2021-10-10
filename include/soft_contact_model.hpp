#ifndef SOFT_CONTACT_MODEL_H
#define SOFT_CONTACT_MODEL_H

#include <Eigen/Dense>
#include <iostream>
#include <math.h>      
#include <mutex>

namespace ContactModel {

template<typename SCALAR>
struct ContactParams {
   SCALAR E;      // Young Modulus
   SCALAR mu;     // friction coefficient
   SCALAR nu;     // possion ratio
   SCALAR R;      // contact tool radius
   SCALAR R_path; // radius of the tracking path
   SCALAR Kd;     // dampning of the surface
};

template<typename SCALAR>
class SoftContactModel {

   ContactParams<SCALAR> m_cp;

public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

   typedef typename Eigen::Matrix<SCALAR, 3, 1> Vector3s;
   typedef typename Eigen::Matrix<SCALAR, 3, 3> Matrix3s;
   std::mutex mu;


SoftContactModel() {
   std::cout << "Initialized Contact Model..." << std::endl;
}

~SoftContactModel() = default;

SoftContactModel(const ContactParams<SCALAR>& cp) : m_cp(cp)
{
   std::cout << "Initialized Contact Model with Params..." << std::endl;
}
   
SoftContactModel(const SoftContactModel<SCALAR>& other)
{
  m_cp = other.m_cp;
}

SoftContactModel& operator=(const SoftContactModel<SCALAR>& other)
{
  this->m_cp = other.m_cp;
  return *this;
}

 /*
  * Soft Contact Modelling Based off Contact Mechanics
  * - states (STATE_DIM parameters)
  * - controls (CONTROL_DIM parameters)
  * - desired end-effector positions (3 parameters)
  * - desired end-effector orientation (9 parameters)
  * - time (1 parameter)
  */
void df(const Eigen::Matrix<SCALAR, 3, 3>& mass_matrix_cart, const Eigen::Matrix<SCALAR, 3, 1>& position, const Eigen::Matrix<SCALAR, 3, 3>& orientation,
   const Eigen::Matrix<SCALAR, 3, 1>& velocity_, const Eigen::Matrix<SCALAR, 3, 1>& acceleration, const Eigen::Matrix<SCALAR, 3, 1>& force_current, Eigen::Matrix<SCALAR, 3, 1>& df_)
{

   std::lock_guard<std::mutex> lk(mu);
   Eigen::Matrix<SCALAR, 3, 1> velDir;

   /* -------------- Normal force calculation -------------- */
   Eigen::Matrix<SCALAR, 3, 1> velNorm(SCALAR(0.0), SCALAR(0.0), SCALAR(0.0000000001));
   Eigen::Matrix<SCALAR, 3, 1> velocity = velocity_ + velNorm;
    velDir = velocity / velocity.norm();
     
   // get the surface normal direction.
   Eigen::Matrix<SCALAR, 3, 1> springDir = surfaceNormal(force_current);

   // the force component in the direction of springDir, projection to spring direction
   Eigen::Matrix<SCALAR, 3, 1> forceZ        = (force_current.dot(springDir) / springDir.norm()) * springDir;
   Eigen::Matrix<SCALAR, 3, 1> velocityZ     = (velocity.dot(springDir) / springDir.norm()) * springDir;
   Eigen::Matrix<SCALAR, 3, 1> accelerationZ = (acceleration.dot(springDir) / springDir.norm()) * springDir;

   // surface deformation resulting from forceZ, dx in the direction of springDir. Using the quasi static model.
   SCALAR d = pow(SCALAR(9.0) * pow(forceZ.norm(), SCALAR(2.0)) / (SCALAR(16.0) * pow(m_cp.E, SCALAR(2)) * m_cp.R), SCALAR(1 / 3));

   // calculate the stiffness
   SCALAR K = pow(SCALAR(6.0) * pow(m_cp.E, SCALAR(2.0)) * m_cp.R * forceZ.norm(), SCALAR(1 / 3)) + SCALAR(600.0);

   // ----------------- Normal force calculation ---------------- 
   Eigen::Matrix<SCALAR, 3, 1> fNDot = K * velocityZ + SCALAR(10.0) * accelerationZ; // Temp


   /* ------------------ Frictional force calculation ---------------- */
   Eigen::Matrix<SCALAR, 3, 1> fFDot = m_cp.mu * K * velocityZ(2) * velDir + SCALAR(3.0) * m_cp.mu * (SCALAR(2.0) * m_cp.nu - SCALAR(1.0)) \
 * (SCALAR(0.0) * d * velDir + forceZ.norm() * velocity) / (SCALAR(10.0) * m_cp.R);


   /* -------------- Orthogonal force calculation -------------- */
   Eigen::Matrix<SCALAR, 3, 1> fNormalDot = SCALAR(2.0) * mass_matrix_cart * velocity.cwiseProduct(acceleration) / m_cp.R_path;

   // df_ = fFDot + fNDot + 0*fNormalDot;

   df_ = fFDot + fNDot;
}


/* implement a way to estimate the surface normal from state data. */
inline Eigen::Matrix<SCALAR, 3, 1> surfaceNormal(const Eigen::Matrix<SCALAR, 3, 1>& force)
{
   Eigen::Matrix<SCALAR, 3, 1> surf(SCALAR(0), SCALAR(0), SCALAR(1));
   return surf;
}

};

}
#endif //SOFT_CONTACT_MODEL_H
