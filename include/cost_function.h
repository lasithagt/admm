#ifndef COSTFUNCTION_H
#define COSTFUNCTION_H

#include "config.h"
#include <iostream>

#include <Eigen/Dense>

using namespace Eigen;


// template <class DynamicsT, class PlantT, class costFunctionT, class OptimizerT, class OptimizerResultT>
class CostFunction
{
	using Jacobian = Eigen::Matrix<double, 1, stateSize + commandSize>;
	using Hessian = Eigen::Matrix<double, 1, stateSize + commandSize>;


public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CostFunction() = default;
    ~CostFunction() = default;

    CostFunction(const stateVec_t &x_goal, const stateVecTab_t &x_track) : x_track_(x_track) {


        Eigen::VectorXd x_w(stateSize);
        Eigen::VectorXd xf_w(stateSize);
        Eigen::VectorXd u_w(commandSize);

        x_w  << 4000, 4000, 4000, 4000, 4000, 4000, 4000, 100, 100, 100, 100, 100, 100, 100, 0, 0, 1000;
        xf_w << 1000, 1000, 1000, 1000, 1000, 1000, 1000, 100, 100, 100, 100, 100, 100, 100, 0, 0, 1000;
        u_w  << 0.05, 0.05, 0.07, 0.07, 0.05, 0.05, 0.05;

        
        Q  = x_w.asDiagonal();
        Qf = xf_w.asDiagonal();
        R  = u_w.asDiagonal();
        

        N = TimeHorizon / TimeStep;
        cx_new.resize(stateSize, N + 1);
        cu_new.resize(commandSize, N + 1);
        cxx_new.resize(N + 1);
        cux_new.resize(N + 1);
        cuu_new.resize(N + 1);

    }

    /* return the cost regular quadratic cost*/
    scalar_t cost_func_expre(const unsigned int& index_k, const stateVec_t& xList_k, const commandVec_t& uList_k)
    {
        scalar_t cost_;
        unsigned int Nl = NumberofKnotPt;

        if (index_k == Nl)
        {
            cost_  = 0.5 * (xList_k.transpose() - x_track_.col(index_k).transpose()) * Qf * (xList_k - x_track_.col(index_k));
        }
        else
        {
            cost_  = 0.5 * (xList_k.transpose() - x_track_.col(index_k).transpose()) * Q * (xList_k - x_track_.col(index_k));
            cost_ += 0.5 * uList_k.transpose() * R * uList_k;
        }

        return cost_;
    }


    /* compute derivatives */
    void computeDerivatives(const stateVecTab_t& xList, const commandVecTab_t& uList)
    {
        // TODO : get the state size from the dynamics class
        unsigned int Nl = xList.cols();

        unsigned int n = stateSize;
        unsigned int m = commandSize;


        for (unsigned int k = 0; k < Nl; k++)
        {

            // Analytical derivatives given quadratic cost
            cx_new.col(k) = Q * (xList.col(k) - x_track_.col(k));
            cu_new.col(k) = R * uList.col(k);

            cxx_new[k] = Q;

            // costFunction->getcux()[k].setZero();
            cuu_new[k] = R; 

            //Note that cu , cux and cuu at the final time step will never be used (see ilqrsolver::doBackwardPass)
            cux_new[k].setZero();
        } 


        c_new = 0;
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


	stateVecTab_t cx_new;
	commandVecTab_t cu_new; 
	stateMatTab_t cxx_new; 
	commandR_stateC_tab_t cux_new; 
	commandMatTab_t cuu_new;
	double c_new;
    unsigned int N;

    stateVecTab_t x_track_;

};




#endif // COSTFUNCTION_H
