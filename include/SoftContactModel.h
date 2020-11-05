#ifndef SOFT_CONTACT_MODEL_H
#define SOFT_CONTACT_MODEL_H

#include <Eigen/Dense>
#include <iostream>
#include <math.h>      

using namespace Eigen;

namespace ContactModel {

struct ContactParams {
   double E;      // Young Modulus
   double mu;     // friction coefficient
   double nu;     // possion ratio
   double R;      // contact tool radius
   double R_path; // radius of the tracking path
   double Kd;     // dampning of the surface
};

class SoftContactModel
{

   ContactParams cp_;

public:
SoftContactModel(){};

SoftContactModel(ContactParams cp) : cp_(cp)
{

   // cp_ = cp;
   // cp_.K = K;

}
   
~SoftContactModel(){}

/* Soft Contact Modelling Based off Contact Mechanics. */
/*  given, end-effector pose, velocity and acceleration in CARTESIAN, returns next force value */

void df(const Eigen::Matrix3d& mass_matrix_cart, const Eigen::Vector3d& position, const Eigen::Vector3d& orientation, 
   const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration, const Eigen::Vector3d& force_current, Eigen::Vector3d& force_next)
{
   // temp arrays
   Eigen::Vector3d vel_dir;


   // Eigen::Vector3d temp;
   // temp(0) = position(0);
   // temp(1) = acceleration(0);
   // temp(2) = force_current(0);
   // temp(2) = orientation(0);

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
   // Eigen::Vector3d spring_dir = surfaceNormal(force_current);
   Eigen::Vector3d spring_dir;
   spring_dir << 0, 0, 1;



   // the force component in the direction of spring_dir, projection to spring direction
   Eigen::Vector3d force_z        = (force_current.dot(spring_dir) / spring_dir.norm()) * spring_dir;
   Eigen::Vector3d velocity_z     = (velocity.dot(spring_dir) /  spring_dir.norm()) * spring_dir;
   Eigen::Vector3d acceleration_z = (acceleration.dot(spring_dir) /  spring_dir.norm()) * spring_dir;


   // surface deformation resulting from force_z, dx in the direction of spring_dir. Using the quasi static model. 
   double d = pow(9 * pow(force_z.norm(), 2) / (16 * pow(cp_.E,2) * cp_.R), 1/3);

   // calulate the stiffness
   double K = pow(6 * pow(cp_.E, 2) * cp_.R * force_z.norm(), (1/3));

   // Eigen::Vector3d F_n_dot = K * velocity_z;
   Eigen::Vector3d F_n_dot = 800.0 * velocity_z + 100.0 * acceleration_z; // Temp

   /* -------------- End - Normal force calculation -------------- */



   /* -------------- Frictional force calculation -------------- */
   // frictional force
   Eigen::Vector3d F_f_dot = cp_.mu * K * velocity_z + 3 * cp_.mu * (2 * cp_.nu - 1) * (K * d * velocity_z  + force_z.norm() * velocity_z) / (10 * cp_.R);// - cp_.Kd * acceleration_z;
     
   /* -------------- End - Frictional force calculation -------------- */



   /* -------------- Orthogonal force calculation -------------- */

   Eigen::Vector3d F_normal_dot = 2 * mass_matrix_cart * velocity.cwiseProduct(acceleration) / cp_.R_path;

   /* -------------- End - Orthogonal force calculation -------------- */



   force_next = 0*F_f_dot - 0*F_n_dot + 0*F_normal_dot;
   // Eigen::Vector3d temp_test;
   // temp_test.setZero();
   // force_next = temp_test;
}


/* implement a way to estimate the surface normal from state data. */
inline Eigen::Vector3d surfaceNormal(const Eigen::Vector3d& force)
{  

   if (force.norm() < 0.1) {
      return force;
   } else {
      return force.normalized();
   }
   
}

};

}
#endif //SOFT_CONTACT_MODEL_H