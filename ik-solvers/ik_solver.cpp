#include <iostream>
#include <Eigen/Dense>
#include <ik_solver.hpp>



IK_FIRST_ORDER::IK_FIRST_ORDER(const Eigen::MatrixXd& Slist_, const Eigen::MatrixXd& M_, const Eigen::MatrixXd& joint_limits_, const double& eomg_, const double& ev_, const Eigen::VectorXd& rho_) {
    
    eomg = eomg_;
    ev = ev_;
    maxIterations = 2;
    rho = rho_;
    Slist = Slist_;
    M = M_;
    cod.setThreshold(0.035);
    joint_limits = joint_limits_;

    q_range =  joint_limits_.row(1) - joint_limits_.row(0);
    q_mid   =  joint_limits_.row(1) / 2 + joint_limits_.row(0) / 2;
}


void IK_FIRST_ORDER::getIK(const Eigen::MatrixXd& Td, const Eigen::VectorXd& thetalist0, const Eigen::VectorXd& thetalistd0, const Eigen::VectorXd& q_bar,
 const Eigen::VectorXd& qd_bar, bool initial, const Eigen::VectorXd& rho, Eigen::VectorXd* thetalist_ret)  {

    Eigen::VectorXd thetalist = thetalist0;

    Tsb = mr::FKinSpace(M, Slist, thetalist0);
    Vs  = mr::Adjoint(Tsb) * mr::se3ToVec(mr::MatrixLog6(mr::TransInv(Tsb) * Td));
    bool err = Vs.head(3).norm() > eomg || Vs.tail(3).norm() > ev;

    if (initial == 1) {
        maxIterations = 50;
    } else {
        maxIterations = 4;
    }

    int i = 0;
    Eigen::MatrixXd J(6, NDOF);

    while (err && i < maxIterations) {

        J = mr::JacobianSpace(Slist, thetalist);
        cod.compute(J);
            
        thetalist = thetalist + 0.5 * cod.solve(Vs) - 0.5 * rho(4) * (thetalist - q_bar);

        Tsb = mr::FKinSpace(M, Slist, thetalist);
        Vs  = mr::Adjoint(Tsb) * mr::se3ToVec(mr::MatrixLog6(mr::TransInv(Tsb) * Td));
    
        /* redundancy resolution */
        //  theta_null = alpha(j) * (eye(7) - J(1:3,:)'*JINV(:,1:3)')  * null_space(Slist, thetalist)';
        
        err = (Vs.head(3).norm() > eomg) || (Vs.tail(3).norm() > ev);
        i = i + 1;
    }

    *(thetalist_ret) = thetalist;

}


void IK_FIRST_ORDER::getRedundancyResolution(const Eigen::VectorXd& thetalist, Eigen::VectorXd* q_grad_ret) {

    (*q_grad_ret) = (thetalist - q_mid);
 
    // dcdq = q_lim_grad;
}




    