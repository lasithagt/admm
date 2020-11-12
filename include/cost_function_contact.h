#ifndef COSTFUNCTIONCONTACT_H
#define COSTFUNCTIONCONTACT_H

#include "config.h"
#include <Eigen/Dense>


/* structure to compute contraints terms */
template <class T, int S, int C>
struct ContactTerms
{
    // Jacobian J_;
    // Differentiable<T, S, C> diff_;
    // Eigen::NumericalDiff<Differentiable<T, S, C>, Eigen::Forward> num_diff_;

    /* --------------------------------------- calculate forward kinematics --------------------------------------------- */
    double* q;
    double* qd;
    double* qdd;

    Eigen::Matrix<double, 3, 3> poseM;
    Eigen::Matrix<double, 3, 3> massMatrix;
    Eigen::Matrix<double, 6, 7> Jac;
    Eigen::Matrix<double, 6, 7> JacDot;

    Eigen::Matrix<double, stateSize, stateSize> CX; 
    Eigen::Matrix<double, stateSize, stateSize> CXX;
    Eigen::Vector3d poseP;
    Eigen::Vector3d vel;
    Eigen::Vector3d accel;
    Eigen::Vector3d contactTerms;

    std::shared_ptr<RobotAbstract> plant;

    ContactTerms() = default;
    ~ContactTerms() 
    {
        delete[] q;
        delete[] qd;
        delete[] qdd;
    }

    ContactTerms(std::shared_ptr<RobotAbstract>& plant_) : plant(plant_) 
    {
        q = new double[7];
        qd = new double[7];
        qdd = new double[7];
        for (int i=0;i<7;i++) {qdd[i] = 0.0;}
    }

    // compute the contact term
    Eigen::Vector3d computeContactTerms(const stateVec_t& x, double R_c)
    {

        memcpy(q, x.head(7).data(), 7 * sizeof(double));
        memcpy(qd, x.segment(7,7).data(), 7 * sizeof(double));
        plant->getForwardKinematics(q, qd, qdd, poseM, poseP, vel, accel, true);

        for (int i=0;i < 3;i++) {
            contactTerms(i) = vel(i) * vel(i) / R_c;
        }
        

        return contactTerms;
    }

    // compute the mass matrix at the end-effector. 
    Eigen::Matrix3d CartesianMassMatrix()
    {
        return massMatrix;
    }   

    // get the contact jabobian
    Eigen::Matrix<double, 6, 7> getContactJacobian(Eigen::VectorXd q)
    {
        plant->getSpatialJacobian(q.data(), Jac); 
        return Jac;
    }

    // get the contact jabobian dot
    Eigen::Matrix<double, 6, 7> getContactJacobianDot(Eigen::VectorXd q)
    {
        plant->getSpatialJacobianDot(q.data(), JacDot); 
        return JacDot;
    }

    // compute the jacobian
    Eigen::Matrix<double, 6, 7> contact_x(Eigen::VectorXd q) 
    {
        CX.block(0,0,7,7) = getContactJacobianDot(q);
        CX.block(7,7,7,7) = getContactJacobian(q);
    }

    // compute the hessian
    Eigen::Matrix<double, 6, 7> contact_xx(Eigen::VectorXd q) 
    {
        CXX.block(0,0,7,7) = getContactJacobianDot(q);
        // CXX.block() = getContactJacobian();
    }

    /* ------------------------------------------------------------------------------------------------------------------- */
};

#endif // COSTFUNCTION_H
