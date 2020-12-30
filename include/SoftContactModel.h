#ifndef SOFT_CONTACT_MODEL_H
#define SOFT_CONTACT_MODEL_H

#include <Eigen/Dense>
#include <iostream>
#include <math.h>      

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
   typedef typename Eigen::Matrix<SCALAR, 3, 1> Vector3s;
   typedef typename Eigen::Matrix<SCALAR, 3, 3> Matrix3s;


SoftContactModel() {
   std::cout << "Initialized Contact Model..." << std::endl;
}

~SoftContactModel() = default;

SoftContactModel(ContactParams<SCALAR> cp) : m_cp(cp)
{}
   
/* Soft Contact Modelling Based off Contact Mechanics. */
/*  given, end-effector pose, velocity and acceleration in CARTESIAN, returns next force value */

void df(const Eigen::Matrix<SCALAR, 3, 3>& mass_matrix_cart, const Eigen::Matrix<SCALAR, 3, 1>& position, const Eigen::Matrix<SCALAR, 3, 1>& orientation, 
   const Eigen::Matrix<SCALAR, 3, 1>& velocity, const Eigen::Matrix<SCALAR, 3, 1>& acceleration, const Eigen::Matrix<SCALAR, 3, 1>& force_current, Eigen::Matrix<SCALAR, 3, 1>& df_)
{
   Eigen::Matrix<SCALAR, 3, 1> vel_dir;


   /* -------------- Normal force calculation -------------- */
   // end-point velocity direction. used to calculate dynamic friction.
   if (velocity.norm() < 0.1)
   {
      vel_dir = velocity;
   }
   else
   {
      vel_dir = velocity.normalized();
   }
     
   // get the surface normal direction.
   Eigen::Matrix<SCALAR, 3, 1> spring_dir = surfaceNormal(force_current);


   // the force component in the direction of spring_dir, projection to spring direction
   Eigen::Matrix<SCALAR, 3, 1> force_z        = (force_current.dot(spring_dir) / spring_dir.norm()) * spring_dir;
   Eigen::Matrix<SCALAR, 3, 1> velocity_z     = (velocity.dot(spring_dir) /  spring_dir.norm()) * spring_dir;
   Eigen::Matrix<SCALAR, 3, 1> acceleration_z = (acceleration.dot(spring_dir) /  spring_dir.norm()) * spring_dir;


   // surface deformation resulting from force_z, dx in the direction of spring_dir. Using the quasi static model. 
   SCALAR d = pow(SCALAR(9.0) * pow(force_z.norm(), SCALAR(2.0)) / (SCALAR(16.0) * pow(m_cp.E, SCALAR(2)) * m_cp.R), SCALAR(1/3));

   // calulate the stiffness
   SCALAR K = pow(SCALAR(6.0) * pow(m_cp.E, SCALAR(2.0)) * m_cp.R * force_z.norm(), SCALAR(1/3)) + SCALAR(600.0);
   // K = 400;


   /* ----------------- Normal force calculation ---------------- */

   Eigen::Matrix<SCALAR, 3, 1> F_n_dot = K * velocity_z + SCALAR(10.0) * acceleration_z; // Temp

   /* -------------- End - Normal force calculation -------------- */



   /* ------------------ Frictional force calculation ---------------- */

   Eigen::Matrix<SCALAR, 3, 1> F_f_dot = m_cp.mu * K * velocity_z(2) * vel_dir + SCALAR(3.0) * m_cp.mu * (SCALAR(2.0) * m_cp.nu - SCALAR(1.0)) \
   * (SCALAR(0.0) * d * vel_dir  + force_z.norm() * velocity) / (SCALAR(10.0) * m_cp.R);
     
   /* -------------- End - Frictional force calculation -------------- */



   /* -------------- Orthogonal force calculation -------------- */

   Eigen::Matrix<SCALAR, 3, 1> F_normal_dot = SCALAR(2.0) * mass_matrix_cart * velocity.cwiseProduct(acceleration) / m_cp.R_path;

   /* -------------- End - Orthogonal force calculation -------------- */


   // df_ = F_f_dot + F_n_dot + 0*F_normal_dot;

   df_ = F_f_dot + F_n_dot;
}


/* implement a way to estimate the surface normal from state data. */
inline Eigen::Matrix<SCALAR, 3, 1> surfaceNormal(const Eigen::Matrix<SCALAR, 3, 1>& force)
{  

   // if (force.norm() < 0.1) {
   //    return force;
   // } else {
   //    return force.normalized();
   // }

   Eigen::Matrix<SCALAR, 3, 1> surf(SCALAR(0), SCALAR(0), SCALAR(1));
   return surf;
   
}

};

}
#endif //SOFT_CONTACT_MODEL_H
