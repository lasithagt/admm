#ifndef COSTFUNCTION_H
#define COSTFUNCTION_H

#include "config.h"
#include <iostream>

#include <Eigen/Dense>

using namespace Eigen;


// template <class DynamicsT, class PlantT, class costFunctionT, class OptimizerT, class OptimizerResultT>
class CostFunctionADMM
{
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
    struct ComputeContactTerms
    {
        Jacobian J_;
        Differentiable<T, S, C> diff_;
        Eigen::NumericalDiff<Differentiable<T, S, C>, Eigen::Forward> num_diff_;

        /* --------------------------------------- calculate forward kinematics --------------------------------------------- */
        double* q;
        Eigen::Matrix<double, 3, 3> poseM;
        Eigen::Matrix<double, 3, 3> massMatrix;
        Eigen::Vector3d poseP;
        Eigen::Vector3d vel;
        Eigen::Vector3d accel;

        // ComputeContactTerms() : diff_([this](const stateVec_t& x, const commandVec_t& u) -> stateVec_t{ return this->kuka_arm_dynamics(x, u); }), num_diff_(diff_) {}


        Eigen::Vector3d operator()(std::shared_ptr<RobotAbstract>& plant_, stateVec_t x)
        {
            // plant.getForwardKinematics(q, qd, qdd, poseM, poseP, vel, accel, true);
            return vel;

        }

        // compute the mass matrix at the end-effector. 
        Eigen::Matrix3d CartesianMassMatrix()
        {
            return massMatrix;
        }   

        /* ------------------------------------------------------------------------------------------------------------------- */
    };


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CostFunctionADMM() = default;
    ~CostFunctionADMM() = default;

    CostFunctionADMM(int time_steps,  std::shared_ptr<RobotAbstract>& plant_) : N(time_steps), plant(plant_) {

        Eigen::VectorXd x_w(stateSize);
        Eigen::VectorXd xf_w(stateSize);
        Eigen::VectorXd u_w(commandSize);

        /* for consensus admm. read it from te main file as a dynamic parameters passing */
        x_w  << 0, 0, 0, 0, 0, 0, 0, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0, 0, 0.05;
        xf_w << 0, 0, 0, 0, 0, 0, 0, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0, 0, 0.05;
        u_w  << 1E-2, 1E-2, 1E-2, 1E-2, 1E-2, 1E-2, 1E-2;

        
        Q  = x_w.asDiagonal();
        Qf = xf_w.asDiagonal();
        R  = u_w.asDiagonal();
        
        cx_new.resize(stateSize, N + 1);
        cu_new.resize(commandSize, N + 1);
        cxx_new.resize(N + 1);
        cux_new.resize(N + 1);
        cuu_new.resize(N + 1);

    }

    /* return the cost without admm terms */
    scalar_t cost_func_expre(const unsigned int& index_k, const stateVec_t& xList_k, const commandVec_t& uList_k, const stateVec_t &x_track)
    {
        scalar_t cost_;

        if (index_k == N)
        {
            cost_ = 0.5 * (xList_k.transpose() - x_track.transpose()) * Qf * (xList_k - x_track);
        }
        else
        {
            cost_ = 0.5 * (xList_k.transpose() - x_track.transpose()) * Q * (xList_k - x_track);
            cost_ += 0.5 * uList_k.transpose() * R * uList_k;
        }
        return cost_;

    }

    /* return the cost with admm */
    scalar_t cost_func_expre_admm(const unsigned int& index_k, const stateVec_t& xList_k, const commandVec_t& uList_k, const stateVec_t &x_track,
     const Eigen::MatrixXd& cList_bar, const stateVec_t& xList_bar, const commandVec_t& uList_bar, const Eigen::VectorXd& thetaList_bar, const Eigen::VectorXd& rho)
    {
        scalar_t cost_;

        // compute the contact terms.
        Eigen::Vector3d contact_terms = ComputeContact(plant, xList_k);

        if (index_k == N) 
        {
            cost_ = 0.5 * (xList_k.transpose() - x_track.transpose()) * Qf * (xList_k - x_track); 
            cost_ += 0.5 * rho(4) * (xList_k.head(7).transpose() - thetaList_bar.transpose()) * (xList_k.head(7) - thetaList_bar);

        } else {
            cost_ = 0.5 * (xList_k.transpose() - x_track.transpose()) * Q * (xList_k - x_track);
            cost_ += 0.5 * rho(0) * (xList_k.head(7).transpose() - xList_bar.head(7).transpose()) * (xList_k - xList_bar).head(7);
            
            // TODO: contact cost term
            /* -------------------------- compute the contact term ----------------------------------*/

            // cost_ += 0.5 * rho(2) * (contact_terms.head(2).transpose() - cList_bar.transpose()) * (contact_terms.head(2) - cList_bar); // temp
            /* --------------------------------------------------------------------------------------*/

            cost_ += 0.5 * rho(4) * (xList_k.head(7).transpose() - thetaList_bar.transpose()) * (xList_k.head(7) - thetaList_bar);

            cost_ += 0.5 * uList_k.transpose() * R * uList_k; 
            cost_ += 0.5 * rho(1) * (uList_k.transpose() - uList_bar.transpose()) * (uList_k - uList_bar);
       }
        return cost_;

    }

    /* compute analytical derivatives */
    void computeDerivatives(const stateVecTab_t& xList, const commandVecTab_t& uList, const stateVecTab_t &x_track,
        const Eigen::MatrixXd& cList_bar, const stateVecTab_t& xList_bar, const commandVecTab_t& uList_bar, const Eigen::MatrixXd& thetaList_bar, const Eigen::VectorXd& rho)
    {
        // TODO : get the state size from the dynamics class
        unsigned int Nl = xList.cols();

        unsigned int n = stateSize;
        unsigned int m = commandSize;


        Eigen::DiagonalMatrix<double, stateSize> m_(rho(0), rho(0), rho(0), rho(0), rho(0), rho(0), rho(0), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        Eigen::DiagonalMatrix<double, stateSize> n_(rho(4), rho(4), rho(4), rho(4), rho(4), rho(4), rho(4), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        Eigen::VectorXd temp(stateSize);

        for (unsigned int k = 0; k < Nl - 1; k++)
        {
            // Analytical derivatives given quadratic cost
            temp.head(7)  = (xList.col(k).head(7) - thetaList_bar.col(k));
            cx_new.col(k) = Q * (xList.col(k) - x_track.col(k)) + m_ * (xList.col(k) - xList_bar.col(k)) + n_ *  temp;
            cu_new.col(k) = R * uList.col(k) + rho(1) * (uList.col(k) - uList_bar.col(k));
            cxx_new[k]    = Q;

            // TODO: contact cost term derivative. Needs num diff
            // compute the first derivative. ignore the second term of te second derivative.

            cxx_new[k]   += m_;
            cxx_new[k]   += n_;
            cuu_new[k]    = R + rho(1) * Eigen::MatrixXd::Identity(7, 7); 

            //Note that cu , cux and cuu at the final time step will never be used (see ilqrsolver::doBackwardPass)
            cux_new[k].setZero();
        } 

        temp.head(7)     = (xList.col(Nl-1).head(7) - thetaList_bar.col(Nl-1));
        cx_new.col(Nl-1) = Q * xList.col(Nl-1) + m_ * (xList.col(Nl-1) - xList_bar.col(Nl-1)) + n_ *  temp; // + rho(0) * (xList.col(Nl) - xList_bar.col(Nl));
        cxx_new[Nl-1]    = Q;
        cxx_new[Nl-1]   += m_;
        cxx_new[Nl-1]   += n_;

        // cuu_new[Nl-1]    = R ; //+ rho(1) * Eigen::MatrixXd::Identity(7, 7); 

        // c_new = 0; // TODO: move this to somewhere.
    }



	const Eigen::Matrix<double,6,6>& getT() const {return T;};

	const stateMat_t& getQ() const {return Q;};

	const stateMat_t& getQf() const {return Qf;};

	const commandMat_t& getR() const {return R;};

	const stateVecTab_t& getcx() const {return cx_new;};

	const commandVecTab_t& getcu() const {return cu_new;};

	const stateMatTab_t& getcxx() const {return cxx_new;};

	const commandR_stateC_tab_t& getcux() const {return cux_new;};

	const commandMatTab_t& getcuu() const {return cuu_new;};

	const double& getc() const {return c_new;};

	
protected:
	stateMat_t Q;
	stateMat_t Qf;
	commandMat_t R;
	Eigen::Matrix<double,6,6> T;

	stateVec_t x_w;
	stateVec_t xf_w;
	commandVec_t u_w;
	Eigen::Matrix<double,6,1> TDiagElementVec;


	stateVecTab_t cx_new;
	commandVecTab_t cu_new; 
	stateMatTab_t cxx_new; 
	commandR_stateC_tab_t cux_new; 
	commandMatTab_t cuu_new;
	double c_new;
    int N;

    // kuka model to get forward kinematics for te cost
    std::shared_ptr<RobotAbstract> plant;

    // structure to compute contact terms
    ComputeContactTerms<double, stateSize, commandSize> ComputeContact;


};




#endif // COSTFUNCTION_H
