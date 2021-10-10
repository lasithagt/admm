#include <Eigen/Dense>
#include <random>
#include <utility>

#include "differential_ik_solver.hpp"


IK_FIRST_ORDER::IK_FIRST_ORDER(const Eigen::MatrixXd& Slist_, const Eigen::MatrixXd& M_, const Eigen::MatrixXd& jointLimits,
    const double& eomg_, const double& ev_, const Eigen::VectorXd& rho_)
{
    
    eomg = eomg_;
    ev = ev_;
    maxIterations = 2;
    rho = rho_;
    Slist = Slist_;
    M = M_;
    cod.setThreshold(0.035);
    joint_limits = jointLimits;

    q_range = jointLimits.row(1) - jointLimits.row(0);
    q_mid   = jointLimits.row(1) / 2 + jointLimits.row(0) / 2;
}


void IK_FIRST_ORDER::getIK(const Eigen::MatrixXd& Td, const Eigen::VectorXd& thetalist0, const Eigen::VectorXd& thetalistd0, const Eigen::VectorXd& q_bar,
 const Eigen::VectorXd& qd_bar, bool initial, const Eigen::VectorXd& rho, Eigen::VectorXd& thetalist_ret)  
{

    Eigen::VectorXd thetalist = thetalist0;

    Tsb = mr::FKinSpace(M, Slist, thetalist0);
    Vs  = mr::Adjoint(Tsb) * mr::se3ToVec(mr::MatrixLog6(mr::TransInv(Tsb) * Td));
    bool err = Vs.head(3).norm() > eomg || Vs.tail(3).norm() > ev;

    if (initial == 1) {
        maxIterations = 50;
    } else {
        maxIterations = 10;
    }

    int i = 0;
    Eigen::MatrixXd J(6, 7);

    while (err && i < maxIterations) {

        J = mr::JacobianSpace(Slist, thetalist);
        cod.compute(J);
            
        thetalist = thetalist + 0.5 * cod.solve(Vs) - 0*0.5 * rho(4) * (thetalist - q_bar);

        Tsb = mr::FKinSpace(M, Slist, thetalist);
        Vs  = mr::Adjoint(Tsb) * mr::se3ToVec(mr::MatrixLog6(mr::TransInv(Tsb) * Td));
    
        /* redundancy resolution */
        //  theta_null = alpha(j) * (eye(7) - J(1:3,:)'*JINV(:,1:3)')  * null_space(Slist, thetalist)';
        
        err = (Vs.head(3).norm() > eomg) || (Vs.tail(3).norm() > ev);
        i = i + 1;
    }

    thetalist_ret = thetalist;

}

void IK_FIRST_ORDER::getIK_random_initial(const Eigen::MatrixXd& Td, const Eigen::VectorXd& q_bar, 
    const Eigen::VectorXd& rho, Eigen::VectorXd& thetalist_ret)  
{
    const int n_random_points = 20;
    std::vector<std::pair<double, Eigen::VectorXd>> diff_store;
    Eigen::VectorXd initialRandomState(thetalist_ret.rows());
    initialRandomState.setZero();

    Eigen::VectorXd thetalistd0 = initialRandomState * 0;
    Eigen::VectorXd qd_bar      = initialRandomState * 0;
    
    for (int i = 0;i < n_random_points;i++)
    {
        // get random joint vectors
        getRandomState(initialRandomState, 1);

        getIK(Td, initialRandomState, thetalistd0, q_bar, qd_bar, true, rho, thetalist_ret);

        // check how far from desired
        double diff = (mr::FKinSpace(M, Slist, thetalist_ret) - Td).norm(); 
        diff_store.emplace_back(diff, thetalist_ret);
        if (diff < 0.001)
        {
            break;
        }

    }
    // otherwise, get the joint positions with the smallest error
    std::sort(diff_store.begin(), diff_store.end(), [](const auto &it1, const auto &it2) {
        return it1.first < it2.first;
    });

    thetalist_ret = diff_store.at(0).second;
}


void IK_FIRST_ORDER::getRandomState(Eigen::VectorXd& randomState, int elbow)
{
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(-1.0, 1.0); 
    std::uniform_real_distribution<> dis_elbow(0.0, 1.0);

    for (int n = 0; n < randomState.rows(); n++) 
    {
        randomState(n) = dis(gen);
    }

    if (elbow == 1) 
    {
        for (auto i : {1,3,5}) {randomState(i) = dis_elbow(gen);}
    }
}

void IK_FIRST_ORDER::getRedundancyResolution(const Eigen::VectorXd& thetalist, Eigen::VectorXd* q_grad_ret) 
{
    (*q_grad_ret) = (thetalist - q_mid);
 
    // dcdq = q_lim_grad;
}


/* Second order motion policies */
IK_SECOND_ORDER::IK_SECOND_ORDER(const Eigen::MatrixXd& Slist_, const Eigen::MatrixXd& M_, const Eigen::MatrixXd& joint_limits_, const double& eomg_, const double& ev_, const Eigen::VectorXd& rho_) {
    
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


void IK_SECOND_ORDER::getIK(const Eigen::MatrixXd& Td, const Eigen::VectorXd& thetalist0, const Eigen::VectorXd& thetalistd0, const Eigen::VectorXd& q_bar,
 const Eigen::VectorXd& qd_bar, bool initial, const Eigen::VectorXd& rho, Eigen::VectorXd* thetalist_ret)  {

    Eigen::VectorXd thetalist = thetalist0;
    Eigen::VectorXd thetalist_d  = thetalistd0;

    Tsb = mr::FKinSpace(M, Slist, thetalist0);
    Vs  = mr::Adjoint(Tsb) * mr::se3ToVec(mr::MatrixLog6(mr::TransInv(Tsb) * Td));
    bool err = Vs.head(3).norm() > eomg || Vs.tail(3).norm() > ev;

    if (initial == 1) {
        maxIterations = 50;
    } else {
        maxIterations = 4;
    }

    int i = 0;
    Eigen::MatrixXd J(6, 7);

    while (err && i < maxIterations) {

        J = mr::JacobianSpace(Slist, thetalist);
        cod.compute(J);
            

        thetalist_d = thetalist_d + cod.solve(Vs - 0.1*J * thetalist_d);
        thetalist   = thetalist + thetalist_d;


        Tsb = mr::FKinSpace(M, Slist, thetalist);
        Vs  = mr::Adjoint(Tsb) * mr::se3ToVec(mr::MatrixLog6(mr::TransInv(Tsb) * Td));
    
        /* redundancy resolution */
        //  theta_null = alpha(j) * (eye(7) - J(1:3,:)'*JINV(:,1:3)')  * null_space(Slist, thetalist)';
        
        err = (Vs.head(3).norm() > eomg) || (Vs.tail(3).norm() > ev);
        i = i + 1;
    }

    *(thetalist_ret) = thetalist;

}


void IK_SECOND_ORDER::getRedundancyResolution(const Eigen::VectorXd& thetalist, Eigen::VectorXd* q_grad_ret) {

    (*q_grad_ret) = (thetalist - q_mid);
 
    // dcdq = q_lim_grad;
}




    