#ifndef SOFT_CONTACT_MODEL_H
#define SOFT_CONTACT_MODEL_H

#include <Eigen/Dense>
#include <iostream>
#include <math.h>      

namespace ContactModel {

struct ContactParams {
   double E;      // Young Modulus
   double mu;     // friction coefficient
   double nu;     // possion ratio
   double R;      // contact tool radius
   double R_path; // radius of the tracking path
   double Kd;     // dampning of the surface
};

class SoftContactModel {

   ContactParams m_cp;

public:
SoftContactModel() {
   std::cout << "Initialized Contact Model..." << std::endl;
}

~SoftContactModel() = default;

SoftContactModel(ContactParams cp) : m_cp(cp)
{}
   
/* Soft Contact Modelling Based off Contact Mechanics. */
/*  given, end-effector pose, velocity and acceleration in CARTESIAN, returns next force value */

void df(const Eigen::Matrix3d& mass_matrix_cart, const Eigen::Vector3d& position, const Eigen::Vector3d& orientation, 
   const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration, const Eigen::Vector3d& force_current, Eigen::Vector3d& force_next)
{
   Eigen::Vector3d vel_dir;


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
   Eigen::Vector3d spring_dir = surfaceNormal(force_current);


   // the force component in the direction of spring_dir, projection to spring direction
   Eigen::Vector3d force_z        = (force_current.dot(spring_dir) / spring_dir.norm()) * spring_dir;
   Eigen::Vector3d velocity_z     = (velocity.dot(spring_dir) /  spring_dir.norm()) * spring_dir;
   Eigen::Vector3d acceleration_z = (acceleration.dot(spring_dir) /  spring_dir.norm()) * spring_dir;


   // surface deformation resulting from force_z, dx in the direction of spring_dir. Using the quasi static model. 
   double d = pow(9 * pow(force_z.norm(), 2) / (16 * pow(m_cp.E,2) * m_cp.R), 1/3);

   // calulate the stiffness
   double K = pow(6 * pow(m_cp.E, 2) * m_cp.R * force_z.norm(), (1/3)) + 600;
   // K = 400;


   /* ----------------- Normal force calculation ---------------- */

   Eigen::Vector3d F_n_dot = K * velocity_z + 10.0 * acceleration_z; // Temp

   /* -------------- End - Normal force calculation -------------- */



   /* ------------------ Frictional force calculation ---------------- */

   Eigen::Vector3d F_f_dot = m_cp.mu * K * velocity_z(2) * vel_dir + 3 * m_cp.mu * (2 * m_cp.nu - 1) * (0 * d * vel_dir  + force_z.norm() * velocity) / (10 * m_cp.R);
     
   /* -------------- End - Frictional force calculation -------------- */



   /* -------------- Orthogonal force calculation -------------- */

   Eigen::Vector3d F_normal_dot = 2 * mass_matrix_cart * velocity.cwiseProduct(acceleration) / m_cp.R_path;

   /* -------------- End - Orthogonal force calculation -------------- */


   force_next = F_f_dot + F_n_dot + 0*F_normal_dot;
}


/* implement a way to estimate the surface normal from state data. */
inline Eigen::Vector3d surfaceNormal(const Eigen::Vector3d& force)
{  

   // if (force.norm() < 0.1) {
   //    return force;
   // } else {
   //    return force.normalized();
   // }

   Eigen::Vector3d surf(0, 0, 1);
   return surf;
   
}

};

}
#endif //SOFT_CONTACT_MODEL_H
