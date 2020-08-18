#ifndef ILQRSOLVER_H
#define ILQRSOLVER_H

#include "config.h"
#include "kuka_arm.h"
#include "cost_function_admm.h"
#include "SoftContactModel.h"

#include <numeric>
#include <sys/time.h>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Cholesky>


#define ENABLE_QPBOX 0
#define DISABLE_QPBOX 1
#define ENABLE_FULLDDP 0
#define DISABLE_FULLDDP 1

#ifndef DEBUG_ILQR
#define DEBUG_ILQR 1
#else
    #if PREFIX1(DEBUG_ILQR)==1
    #define DEBUG_ILQR 1
    #endif
#endif

#define TRACE(x) do { if (DEBUG_ILQR) printf(x);} while (0)

using namespace Eigen;

namespace optimizer {


class ILQRSolverADMM
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct traj
    {
        stateVecTab_t xList;
        commandVecTab_t uList;
        unsigned int iter;
        double finalCost;
        double finalGrad;
        double finalLambda;
        Eigen::VectorXd time_forward, time_backward, time_derivative; //computation time?
    };

    struct OptSet {
        int n_hor;
        int debug_level;
        stateVec_t xInit;
        double new_cost, cost, dcost, lambda, dlambda, g_norm, expected;
        double **p;
        const double *alpha;
        int n_alpha;
        double lambdaMin, lambdaMax;
        double lambdaInit, dlambdaInit;
        double lambdaFactor;
        unsigned int max_iter;
        double tolGrad, tolFun, tolConstraint;

        double zMin;
        int regType;
        int iterations;
        int *log_linesearch;
        double *log_z;
        double *log_cost;
        double dV[2];
        
        int print;
        double print_head; // print headings every print_head lines
        double last_head;
        Eigen::VectorXd time_backward, time_forward, time_derivative;
        Eigen::VectorXd alphaList;

        OptSet() : debug_level(2), n_alpha(11), lambdaMin(1e-6), lambdaMax(1e10), lambdaInit(1), dlambdaInit(1), lambdaFactor(1.3), max_iter(500), 
                    tolGrad(1e-4), tolFun(1e-4), tolConstraint(1e-7), zMin(0.0), regType(1), print(2) {}

    };


private:
    KukaArm* dynamicModel;
    CostFunctionADMM* costFunction;
    unsigned int stateNb;
    unsigned int commandNb;

    unsigned int N;
    unsigned int iter;
    double dt;
    // commandVecTab_t initCommand;

    stateVecTab_t xList; // vector/array of stateVec_t = basically knot config over entire time horizon
    commandVecTab_t uList;
    // stateVecTab_t cList_bar;
    // stateVecTab_t xList_bar;
    // commandVecTab_t uList_bar;

    commandVecTab_t uListFull;
    commandVec_t u_NAN; //matrix of <commandsize, 1> = essentially a vector
    stateVecTab_t updatedxList;
    commandVecTab_t updateduList;
    stateVecTab_t FList;
    costVecTab_t costList;
    costVecTab_t costListNew;
    struct traj lastTraj;
    struct timeval tbegin_time_fwd, tend_time_fwd, tbegin_time_bwd, tend_time_bwd, tbegin_time_deriv, tend_time_deriv;

    stateVecTab_t Vx;
    stateMatTab_t Vxx;

    stateVec_t Qx;
    stateMat_t Qxx;
    commandVec_t Qu;
    commandMat_t Quu;
    commandMat_t QuuF;
    commandMat_t QuuInv;
    commandR_stateC_t Qux;
    commandVec_t k;
    commandR_stateC_t K;
    commandVecTab_t kList;
    commandR_stateC_tab_t KList;
    double alpha;

    stateMat_t lambdaEye;
    int backPassDone;
    int fwdPassDone;
    int initFwdPassDone;
    int diverge;

    /* QP variables */
    bool enableQPBox;
    bool enableFullDDP;
    commandMat_t H;
    commandVec_t g;
    commandVec_t lowerCommandBounds;
    commandVec_t upperCommandBounds;
    commandVec_t lb;
    commandVec_t ub;

    OptSet Op;
    Eigen::Vector2d dV;
    bool debugging_print;    
    int newDeriv;
    double g_norm_i, g_norm_max, g_norm_sum;
    bool isUNan;

public:
    ILQRSolverADMM(KukaArm& DynamicModel, CostFunctionADMM& Cost, const OptSet& solverOptions, const int& time_steps, const double& dt_, bool fullDDP, bool QPBox);
    void solve(const stateVec_t& x_0, const commandVecTab_t& u_0, const Eigen::MatrixXd& cList_bar, 
        const stateVecTab_t& xList_bar, const commandVecTab_t& uList_bar, const Eigen::MatrixXd& thetaList_bar, const Eigen::VectorXd& rho);
    void initializeTraj(const stateVec_t& x_0, const commandVecTab_t& u_0, const Eigen::MatrixXd& cList_bar, const stateVecTab_t& xList_bar,
     const commandVecTab_t& uList_bar, const Eigen::MatrixXd& thetaList_bar, const Eigen::VectorXd& rho);
    struct traj getLastSolvedTrajectory();

private:
    inline stateVec_t forward_integration(const stateVec_t& X, const commandVec_t& U);
    void doBackwardPass();
    void doForwardPass(const stateVec_t& x_0, const Eigen::MatrixXd& cList_bar, const stateVecTab_t& xList_bar, const commandVecTab_t& uList_bar, const Eigen::MatrixXd& thetaList_bar, const Eigen::VectorXd& rho);
    bool isPositiveDefinite(const commandMat_t & Quu); 
};


} // namesapce


#endif // ILQRSOLVER_H

