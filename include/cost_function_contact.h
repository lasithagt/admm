#ifndef COSTFUNCTIONCONTACT_H
#define COSTFUNCTIONCONTACT_H

#include "config.h"
#include <Eigen/Dense>


using Jacobian = Eigen::Matrix<double, 1, stateSize + commandSize>;
using Hessian = Eigen::Matrix<double, 1, stateSize + commandSize>;

/* numerical derivative computation*/
template <class T, int S, int C>
struct Differentiable
{
    /*****************************************************************************/
    /*** Replicate Eigen's generic functor implementation to avoid inheritance ***/
    /*** We only use the fixed-size functionality ********************************/
    /*****************************************************************************/
    enum { InputsAtCompileTime = S + C, ValuesAtCompileTime = S };
    using Scalar        = T;
    using InputType     = Eigen::Matrix<T, InputsAtCompileTime, 1>;
    using ValueType     = Eigen::Matrix<T, ValuesAtCompileTime, 1>;
    using JacobianType  = Eigen::Matrix<T, ValuesAtCompileTime, InputsAtCompileTime>;
    int inputs() const { return InputsAtCompileTime; }
    int values() const { return ValuesAtCompileTime; }
    int operator()(const Eigen::Ref<const InputType> &xu, Eigen::Ref<ValueType> dx) const
    {
        dx =  dynamics_(xu.template head<S>(), xu.template tail<C>());
        return 0;
    }
    /*****************************************************************************/

    using DiffFunc = std::function<Eigen::Matrix<T, S, 1>(const Eigen::Matrix<T, S, 1>&, const Eigen::Matrix<T, C, 1>&)>;
    Differentiable(const DiffFunc &dynamics) : dynamics_(dynamics) {}
    Differentiable() = default;

private:
    DiffFunc dynamics_;
};

/* structure to compute contraints terms */
template <class T, int S, int C>
struct ContactTerms
{

    /* --------------------------------------- calculate forward kinematics --------------------------------------------- */
    double* q;
    double* qd;
    double* qdd;

    Eigen::Matrix<double, 3, 3> poseM;
    Eigen::Matrix<double, 3, 3> massMatrix;
    Eigen::MatrixXd Jac;
    Eigen::MatrixXd JacDot;

    Eigen::VectorXd CX; 
    Eigen::MatrixXd CXX;
    Eigen::Vector3d poseP;
    Eigen::Vector3d vel;
    Eigen::Vector3d accel;
    Eigen::Vector2d contactTerms;
    double mass;

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
        q   = new double[7];
        qd  = new double[7];
        qdd = new double[7];
        Jac.resize(6, NDOF);
        JacDot.resize(6, NDOF);
        CX.resize(stateSize);
        CXX.resize(stateSize, stateSize);

        for (int i=0;i<7;i++) {qdd[i] = 0.0;}
        mass = 0.3;
    }

    // compute the contact term
    Eigen::Vector2d computeContactTerms(const stateVec_t& x, double R_c)
    {

        // get the path parameters. K on top of R_c
        memcpy(q, x.head(7).data(), 7 * sizeof(double));
        memcpy(qd, x.segment(7,7).data(), 7 * sizeof(double));
        plant->getForwardKinematics(q, qd, qdd, poseM, poseP, vel, accel, true);

        contactTerms(0) = mass * (vel.transpose() * vel)(0) / R_c;

        contactTerms(1) = x(16);

        return contactTerms;
    }

    /* compute the mass matrix at the end-effector. */
    inline Eigen::Matrix3d CartesianMassMatrix()
    {
        return massMatrix;
    }   

    /* get the contact jabobian */
    inline Eigen::MatrixXd getContactJacobian(double* q)
    {
        plant->getSpatialJacobian(q, Jac); 
        return Jac;
    }

    /* get the contact jabobian dot */
    inline Eigen::MatrixXd getContactJacobianDot(double* q, double* qd)
    {
        plant->getSpatialJacobianDot(q, q, JacDot); 
        return JacDot;
    }

    /* compute the jacobian, assuming contact terms are calculated first */
    Eigen::VectorXd contact_x(const stateVec_t& x, const Eigen::VectorXd& cList_bar, double R_c, double rho_c) 
    {
        // TODO: optimize this part
        memcpy(q, x.head(7).data(), 7 * sizeof(double));
        memcpy(qd, x.segment(7,7).data(), 7 * sizeof(double));

        plant->getForwardKinematics(q, qd, qdd, poseM, poseP, vel, accel, true);

        Eigen::Vector2d w;
        w = (computeContactTerms(x, R_c) - cList_bar);

        CX.head(7)      = rho_c * 2 * mass * (w(0) + w(1)) * (1/R_c) * vel.transpose() * getContactJacobianDot(q, qd).block(0,0,3,NDOF);
        CX.segment(7,7) = rho_c * 2 * mass * (w(0) + w(1)) * (1/R_c) * vel.transpose() * getContactJacobian(q).block(0,0,3,NDOF);


        return CX;
    }

    /* compute the hessian */
    Eigen::MatrixXd contact_xx(const stateVec_t& x, const Eigen::VectorXd& cList_bar, double R_c, double rho_c) 
    {
        // TODO: optimize this part
        // Assumption: 
        memcpy(q, x.head(7).data(), 7 * sizeof(double));
        memcpy(qd, x.segment(7,7).data(), 7 * sizeof(double));

        plant->getForwardKinematics(q, qd, qdd, poseM, poseP, vel, accel, true);

        Eigen::Vector2d w;
        w = (computeContactTerms(x, R_c) - cList_bar);

        CXX.block(0,0,7,7) = rho_c * 2 * mass * (1.0/R_c) * (w(0) + w(1)) * getContactJacobianDot(q, qd).block(0,0,3,NDOF).transpose() * getContactJacobianDot(q, qd).block(0,0,3,NDOF) + \
                                rho_c * 2 * getContactJacobianDot(q, qd).block(0,0,3,NDOF).transpose() * vel * mass * (1/R_c) * vel.transpose() * getContactJacobianDot(q, qd).block(0,0,3,NDOF);

        CXX.block(7,7,7,7) = rho_c * 2 * mass  * (1.0/R_c) * (w(0) + w(1)) * getContactJacobian(q).block(0,0,3,NDOF).transpose() * getContactJacobian(q).block(0,0,3,NDOF) + \
                                rho_c * 2 * getContactJacobian(q).block(0,0,3,NDOF).transpose() * vel * mass * (1/R_c) * vel.transpose() * getContactJacobian(q).block(0,0,3,NDOF);

        CXX.block(0,7,7,7) = rho_c * 2 * mass * (1.0/R_c) * (w(0) + w(1)) * getContactJacobian(q).block(0,0,3,NDOF).transpose() * getContactJacobianDot(q, qd).block(0,0,3,NDOF) + \
                                rho_c * 2 * mass * getContactJacobian(q).block(0,0,3,NDOF).transpose() * vel * (1/R_c) * vel.transpose() * getContactJacobian(q).block(0,0,3,NDOF);
        
        CXX.block(7,0,7,7) = CXX.block(0,7,7,7).transpose();

        Eigen::DiagonalMatrix<double, 3> fxx(0,0,rho_c);

        CXX.block(14,14,3,3) = fxx;
        return CXX;
    }



    /* ------------------------------------------------------------------------------------------------------------------- */
};

#endif // COSTFUNCTIONCONTACT_H

