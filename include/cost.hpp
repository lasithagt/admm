#ifndef COST_H
#define COST_H

#include <iostream>
#include <memory>

template <class DynamicsT, class PlantT, class costFunctionT, class OptimizerT, class OptimizerResultT>
class Cost
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

    Cost() = default;
    ~Cost() = default;

    Cost(int time_steps,  const std::shared_ptr<RobotAbstract>& robotModel) : N(time_steps), plant(robotModel) {

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

    }



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



    /* return the cost with admm */
    Scalar cost_func_expre_admm(unsigned int k, const State& x_k, const Control& u_k, const State &x_track,
                                const Eigen::MatrixXd& c_bar, const State& x_bar, const Control& u_bar, 
                                const Eigen::VectorXd& thetaList_bar, const Eigen::VectorXd& rho, const Eigen::VectorXd& R_c)
    {

    }



    /* compute analytical derivatives */
    void computeDerivatives(const StateTrajectory& xList, const ControlTrajectory& uList, const StateTrajectory &x_track,
                            const Eigen::MatrixXd& cList_bar, const StateTrajectory& xList_bar, const ControlTrajectory& uList_bar, 
                            const Eigen::MatrixXd& thetaList_bar, const Eigen::VectorXd& rho, const Eigen::VectorXd& R_c)
    {
 
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
