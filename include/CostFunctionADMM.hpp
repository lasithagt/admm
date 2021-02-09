#ifndef COSTFUNCTION_H
#define COSTFUNCTION_H

#include "config.h"
#include "CostFunctionContact.hpp"
#include <memory>

// template <class DynamicsT, class PlantT, class costFunctionT, class OptimizerT, class OptimizerResultT>
class CostFunctionADMM
{
    using Scalar               = scalar_t;
    using State                = stateVec_t;
    using Control              = commandVec_t;
    using StateTrajectory      = stateVecTab_t;
    using ControlTrajectory    = commandVecTab_t;
    using ControlStateJacobian = commandR_stateC_tab_t;
    using StateHessian         = stateMat_t;
    using ControlHessian       = commandMat_t;
    using StateWeights         = stateMat_t;
    using ControlWeights       = commandMat_t;

protected:
    StateWeights Q;
    StateWeights Qf;
    ControlWeights R;

    State x_w;
    State xf_w;
    Control u_w;

    StateTrajectory cx_new;
    ControlTrajectory cu_new; 

    stateMatTab_t cxx_new; 
    ControlStateJacobian cux_new; 
    commandMatTab_t cuu_new;
    Scalar c_new;
    int N;

    // kuka model to get forward kinematics for te cost
    std::shared_ptr<RobotAbstract> plant;

    // structure to compute contact terms
    // std::shared_ptr<ContactTerms<double, stateSize, commandSize>> m_contactCost;
    ContactTerms<double, stateSize, commandSize>* m_contactCost;

    Eigen::Matrix<double, stateSize, 1> m_;
    Eigen::Matrix<double, stateSize, 1> n_;

    Eigen::Matrix<double, stateSize, stateSize> c_xx;
    Eigen::Matrix<double, stateSize, 1> c_x;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CostFunctionADMM() = default;
    ~CostFunctionADMM() 
    {
        delete m_contactCost;
    }

    CostFunctionADMM(int time_steps,  const std::shared_ptr<RobotAbstract>& robotModel) : N(time_steps), plant(robotModel) {

        Eigen::VectorXd x_w(stateSize);
        Eigen::VectorXd xf_w(stateSize);
        Eigen::VectorXd u_w(commandSize);

        /* for consensus admm. read it from te main file as a dynamic parameters passing */
        x_w  << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5;
        xf_w << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5;
        u_w  << 1E-4, 1E-4, 1E-4, 1E-4, 1E-4, 1E-4, 1E-4;

        
        Q  = x_w.asDiagonal();
        Qf = xf_w.asDiagonal();
        R  = u_w.asDiagonal();
        
        cx_new.resize(stateSize, N + 1);
        cu_new.resize(commandSize, N + 1);
        cxx_new.resize(N + 1);
        cux_new.resize(N + 1);
        cuu_new.resize(N + 1);

        Eigen::VectorXd temp(stateSize);

        m_contactCost = new ContactTerms<double, stateSize, commandSize>(robotModel);

    }

    CostFunctionADMM(const CostFunctionADMM& other) {}

    CostFunctionADMM& operator = (const CostFunctionADMM& other) {}


    /* return the cost without admm terms */
    Scalar cost_func_expre(unsigned int k, const State& x_k, const Control& u_k, const State &x_track)
    {
        Scalar cost;

        if (k == N)
        {
            cost = 0.5 * (x_k.transpose() - x_track.transpose()) * Qf * (x_k - x_track);
        }
        else
        {
            cost  = 0.5 * (x_k.transpose() - x_track.transpose()) * Q * (x_k - x_track);
            cost += 0.5 * u_k.transpose() * R * u_k;
        }
        return cost;
    }



    // return the cost with admm 
    Scalar cost_func_expre_admm(unsigned int k, const State& x_k, const Control& u_k, const State &x_track,
                                const Eigen::MatrixXd& c_bar, const State& x_bar, const Control& u_bar, 
                                const Eigen::VectorXd& thetaList_bar, const Eigen::VectorXd& rho, const Eigen::VectorXd& R_c)
    {
        Scalar cost;

        // compute the contact terms.
        Eigen::Vector2d contact_terms = m_contactCost->computeContactTerms(x_k, R_c(k));

        if (k == N) 
        {
            cost  = 0.5 * (x_k.transpose() - x_track.transpose()) * Qf * (x_k - x_track); 
            cost += 0.5 * rho(4) * (x_k.head(7).transpose() - thetaList_bar.transpose()) * (x_k.head(7) - thetaList_bar);
            
            if (CONTACT_EN)
            {
                cost += 0.5 * rho(2) * (contact_terms.head(2).transpose() - c_bar.transpose()) * (contact_terms.head(2) - c_bar); // temp
            }  
            cost += 0.5 * rho(0) * (x_k.head(7).transpose() - x_bar.head(7).transpose()) * (x_k - x_bar).head(7);

        } else {

            cost  = 0.5 * (x_k.transpose() - x_track.transpose()) * Q * (x_k - x_track);
            cost += 0.5 * u_k.transpose() * R * u_k; 

            cost += 0.5 * rho(0) * (x_k.head(7).transpose() - x_bar.head(7).transpose()) * (x_k - x_bar).head(7);
            cost += 0.5 * rho(1) * (u_k.transpose() - u_bar.transpose()) * (u_k - u_bar);


            // compute the contact term 
            if (CONTACT_EN)
            {
                cost += 0.5 * rho(2) * (contact_terms.head(2).transpose() - c_bar.transpose()) * (contact_terms.head(2) - c_bar); // temp
            }   

            cost += 0.5 * rho(4) * (x_k.head(7).transpose() - thetaList_bar.transpose()) * (x_k.head(7) - thetaList_bar);
            
        }

        return cost;
    }



    /* compute analytical derivatives */
    void computeDerivatives(const StateTrajectory& xList, const ControlTrajectory& uList, const StateTrajectory &x_track,
                            const Eigen::MatrixXd& cList_bar, const StateTrajectory& xList_bar, const ControlTrajectory& uList_bar, 
                            const Eigen::MatrixXd& thetaList_bar, const Eigen::VectorXd& rho, const Eigen::VectorXd& R_c)
    {
        // TODO : get the state size from the dynamics class

        m_ << rho(0), rho(0), rho(0), rho(0), rho(0), rho(0), rho(0), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        n_ << rho(4), rho(4), rho(4), rho(4), rho(4), rho(4), rho(4), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        Eigen::VectorXd temp(stateSize);

        for (unsigned int k = 0; k < N; k++)
        {
            if (CONTACT_EN)
            {
                c_x  = m_contactCost->contact_x(xList.col(k), cList_bar.col(k), R_c(k), rho(2));
                c_xx = m_contactCost->contact_xx(xList.col(k), cList_bar.col(k), R_c(k), rho(2));
                cx_new.col(k) = Q * (xList.col(k) - x_track.col(k)) + m_.asDiagonal() * (xList.col(k) - xList_bar.col(k)) + n_.asDiagonal() *  temp + c_x;
            } else
            {
                cx_new.col(k) = Q * (xList.col(k) - x_track.col(k)) + m_.asDiagonal() * (xList.col(k) - xList_bar.col(k)) + n_.asDiagonal() *  temp;
            }

            // Analytical derivatives given quadratic cost
            temp.head(7)  = (xList.col(k).head(7) - thetaList_bar.col(k));
            cu_new.col(k) = R * uList.col(k) + rho(1) * (uList.col(k) - uList_bar.col(k));
            

            // compute the first derivative. ignore the second term of te second derivative.
            cxx_new[k]    = Q;
            cxx_new[k]   += m_.asDiagonal();
            cxx_new[k]   += n_.asDiagonal();

            if (CONTACT_EN) {cxx_new[k] += c_xx;}
            cuu_new[k]    = R + rho(1) * Eigen::MatrixXd::Identity(7, 7); 

            // Note that cu , cux and cuu at the final time step will never be used (see ilqrsolver::doBackwardPass)
            cux_new[k].setZero();
        } 

        temp.head(7)  = (xList.col(N).head(7) - thetaList_bar.col(N));

        if (CONTACT_EN)
        {
            c_x = m_contactCost->contact_x(xList.col(N), cList_bar.col(N), R_c(N), rho(2));
            cx_new.col(N) = Q * (xList.col(N) - x_track.col(N)) + m_.asDiagonal() * (xList.col(N) - xList_bar.col(N)) + n_.asDiagonal() *  temp + c_x; // + rho(0) * (xList.col(N) - xList_bar.col(N));
        } else 
        {
            cx_new.col(N) = Q * (xList.col(N) - x_track.col(N)) + m_.asDiagonal() * (xList.col(N) - xList_bar.col(N)) + n_.asDiagonal() *  temp;
        }

        cxx_new[N]    = Q;
        cxx_new[N]   += m_.asDiagonal();
        cxx_new[N]   += n_.asDiagonal();
    }

	const StateWeights& getQ() const {return Q;};
	const StateWeights& getQf() const {return Qf;};
	const ControlWeights& getR() const {return R;};
	const StateTrajectory& getcx() const {return cx_new;};
	const ControlTrajectory& getcu() const {return cu_new;};
	const stateMatTab_t& getcxx() const {return cxx_new;};
	const ControlStateJacobian& getcux() const {return cux_new;};
	const commandMatTab_t& getcuu() const {return cuu_new;};
	Scalar getc() const {return c_new;};

};

#endif // COSTFUNCTION_H
