
#include "kuka_model.h"


KUKAModelKDL::KUKAModelKDL(const KDL::Chain& robotChain, const KUKAModelKDLInternalData& robotParams) : robotChain_(robotChain), robotParams_(robotParams) 
{
    dynamicsChain_ = new KDL::ChainDynParam(robotChain_, KDL::Vector(0,0,-9.8));
} 

KUKAModelKDL::~KUKAModelKDL()
{
    delete dynamicsChain_;
}

int KUKAModelKDL::initRobot() 
{
    q_.resize(7);
    qd_.resize(7);
    qdd_.resize(7);
    inertia_mat_.resize(7);
    coriolis_.resize(7);
    gravity_.resize(7);
    jacobian_.resize(7);

    robotParams_.Kv = Eigen::MatrixXd::Zero(7, 7);
    robotParams_.Kv.diagonal() << 3.7, 1.5, 1.7, 1.9, 2, 2, 2;


    return true;
}

void KUKAModelKDL::getForwardKinematics(double* q, double* qd, double *qdd, Eigen::Matrix<double,3,3>& poseM, Eigen::Vector3d& poseP, Eigen::Vector3d& vel, Eigen::Vector3d& accel, bool computeOther)
{
    memcpy(q_.data.data(), q, 7 * sizeof(double));
    memcpy(qd_.data.data(), qd, 7 * sizeof(double));
    memcpy(qdd_.data.data(), qdd, 7 * sizeof(double));

    // TODO: make it updatable
    Eigen::MatrixXd T(3,3);
    T << -1, 0, 0, 0, -1, 0, 0, 0, 1;

    if (computeOther)
    {
        // compute pose, vel
        KDL::ChainFkSolverVel_recursive fksolver_vel(robotChain_);
        KDL::JntArrayVel jntVel(q_, qd_);
        fksolver_vel.JntToCart(jntVel, frame_vel_, -1);
        memcpy(poseM.data(), frame_vel_.M.R.data, 9 * sizeof(double));
        memcpy(poseP.data(), frame_vel_.p.p.data, 3 * sizeof(double));
        memcpy(vel.data(), frame_vel_.p.v.data, 3 * sizeof(double));
        poseM.transposeInPlace();

        poseM = poseM.eval() * T; 
        // poseP = T * poseP.eval();

        // compute accel
        KDL::ChainJntToJacSolver jacSolver(robotChain_);
        jacSolver.JntToJac(q_, jacobian_);
        // accel = std::move(jacobian_.data * qdd_.data);
        
        KDL::ChainJntToJacDotSolver jacDotSolver(robotChain_);
        jacDotSolver.JntToJacDot(jntVel, jacobian_, -1);
        // accel += jacobian_.data * qd_.data;


    } else 
    {
        // compute pose
        KDL::ChainFkSolverPos_recursive fksolver_pos(robotChain_);
        fksolver_pos.JntToCart(q_, frame_, -1); 
        memcpy(poseM.data(), frame_.M.data, 9 * sizeof(double));
        memcpy(poseP.data(), frame_.p.data, 3 * sizeof(double));
        poseM.transposeInPlace();

        poseM = poseM.eval() * T; 
        // poseP = T * poseP.eval();
    }
}

/* given q, qdot, qddot, outputs torque output*/
void KUKAModelKDL::getInverseDynamics(double* q, double* qd, double* qdd, Eigen::VectorXd& torque)
{

}

void KUKAModelKDL::getForwardDynamics(double* q, double* qd, const Eigen::VectorXd& force_ext, Eigen::VectorXd& qdd)
{

    memcpy(q_.data.data(), q, 7 * sizeof(double));
    memcpy(qd_.data.data(), qd, 7 * sizeof(double));
    
    dynamicsChain_->JntToMass(q_, inertia_mat_);
    dynamicsChain_->JntToCoriolis(q_, qd_, coriolis_);
    dynamicsChain_->JntToGravity(q_, gravity_);

    Eigen::LLT<Eigen::Ref<Eigen::MatrixXd> > llt(inertia_mat_.data); // Do LU decomposition. This is faster than the SVD approach.


    qdd = std::move(llt.solve(force_ext - robotParams_.Kv * qd_.data - coriolis_.data + 0 * gravity_.data)); 
}

void KUKAModelKDL::getMassMatrix(double* q, Eigen::MatrixXd& massMatrix)
{
    memcpy(q_.data.data(), q, 7 * sizeof(double));
    dynamicsChain_->JntToMass(q_, inertia_mat_);
    massMatrix = std::move(inertia_mat_.data);
}

void KUKAModelKDL::getCoriolisMatrix(double* q, double* qd, Eigen::VectorXd& coriolis) // change
{
    memcpy(q_.data.data(), q, 7 * sizeof(double));
    memcpy(qd_.data.data(), qd, 7 * sizeof(double));
    dynamicsChain_->JntToCoriolis(q_, qd_, coriolis_);
    coriolis = std::move(coriolis_.data);
}

void KUKAModelKDL::getGravityVector(double* q, Eigen::VectorXd& gravityTorque)
{
    memcpy(q_.data.data(), q, 7 * sizeof(double));
    dynamicsChain_->JntToGravity(q_, gravity_);
    gravityTorque = std::move(gravity_.data);
}

void KUKAModelKDL::getSpatialJacobian(double* q, Eigen::MatrixXd& jacobian)
{   
    memcpy(q_.data.data(), q, 7 * sizeof(double));
    KDL::ChainJntToJacSolver jacSolver(robotChain_);
    jacSolver.JntToJac(q_, jacobian_);
    jacobian = std::move(jacobian_.data);
}

void KUKAModelKDL::getSpatialJacobianDot(double* q, double* qd, Eigen::MatrixXd& jacobianDot)
{   
    memcpy(q_.data.data(), q, 7 * sizeof(double));
    memcpy(qd_.data.data(), qd, 7 * sizeof(double));
    KDL::ChainJntToJacDotSolver jacDotSolver(robotChain_);

    KDL::JntArrayVel jntVel(q_, qd_);

    jacDotSolver.JntToJacDot(jntVel, jacobian_, -1);
    jacobianDot = std::move(jacobian_.data);
}

void KUKAModelKDL::ik()
{
    
}




