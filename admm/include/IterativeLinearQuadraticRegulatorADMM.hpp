#ifndef ILQRSOLVER_H
#define ILQRSOLVER_H

#include "config.h"
#include "RobotDynamics.hpp"
#include "CostFunctionADMM.hpp"
#include "SoftContactModel.h"

#include <numeric>
#include <sys/time.h>
#include <mutex>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Cholesky>
#include <chrono>

/* Debug */
#include <iostream>
using namespace std;
/* */

using namespace Eigen;
using Eigen::VectorXd;

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

namespace optimizer {

class IterativeLinearQuadraticRegulatorADMM {
    using State             = stateVec_t;
    using Control           = commandVec_t;
    using StateTrajectory   = stateVecTab_t;
    using ControlTrajectory = commandVecTab_t;
    using ControlStateGains = commandR_stateC_tab_t;
    using ControlHessian    = commandMat_t;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct traj
    {
        stateVecTab_t xList;
        commandVecTab_t uList;
        commandR_stateC_tab_t KList;
        commandVecTab_t kList;
        unsigned int iter;
        double finalCost;
        double finalGrad;
        double finalLambda;
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
        Eigen::VectorXd alphaList;

        OptSet() : debug_level(2), n_alpha(3), lambdaMin(1e-6), lambdaMax(1e10), lambdaInit(1), dlambdaInit(1), lambdaFactor(1.3), max_iter(500), 
                    tolGrad(1e-4), tolFun(1e-4), tolConstraint(1e-7), zMin(0.0), regType(1), print(2) {}

    };


private:
    using Dynamics = admm::Dynamics<RobotAbstract, stateSize, commandSize>;

    std::shared_ptr<Dynamics> dynamicModel;
    std::shared_ptr<CostFunctionADMM> costFunction;
    unsigned int stateNb, commandNb;

    unsigned int N;
    unsigned int iter;
    double dt;

    stateVecTab_t xList; // vector/array of stateVec_t = basically knot config over entire time horizon
    commandVecTab_t uList;

    stateVec_t x_dot1, x_dot2, x_dot3, x_dot4;

    commandVecTab_t uListFull;
    commandVec_t u_NAN; 
    stateVecTab_t updatedxList;
    commandVecTab_t updateduList;
    stateVecTab_t FList;
    costVecTab_t costList, costListNew;
    struct traj lastTraj;

    stateVecTab_t Vx;
    stateMatTab_t Vxx;

    stateVec_t Qx;
    stateMat_t Qxx;
    commandVec_t Qu;
    commandMat_t Quu, QuuF, QuuInv;
    commandR_stateC_t Qux;
    commandVec_t k;
    commandR_stateC_t K;
    commandVecTab_t kList;
    commandR_stateC_tab_t KList;
    double alpha;

    int backPassDone, fwdPassDone, initFwdPassDone, diverge;

    /* QP variables */
    bool enableQPBox;
    bool enableFullDDP;
    commandMat_t H;
    commandVec_t g;
    commandVec_t lowerCommandBounds, upperCommandBounds, lb, ub;

    Control u_NAN_loc;

    OptSet Op;
    Eigen::Vector2d dV;
    bool debugging_print;    
    int newDeriv;
    double g_norm_i, g_norm_max, g_norm_sum;
    bool isUNan;

    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    std::chrono::duration<float, std::nano> elapsed;

public:
    IterativeLinearQuadraticRegulatorADMM()
    {
        std::cout << "Initialized the Optimizer Model..." << std::endl;

    }
    
    IterativeLinearQuadraticRegulatorADMM(const std::shared_ptr<Dynamics>& DynamicModel, const std::shared_ptr<CostFunctionADMM>& CostFunction, 
        const OptSet& solverOptions, int time_steps, double dt_, bool fullDDP, bool QPBox) : 
        dynamicModel(DynamicModel), costFunction(CostFunction), N(time_steps), dt(dt_), Op(solverOptions)
    {
        // dynamicModel  = &DynamicModel;
        // costFunction  = &CostFunction;

        enableQPBox   = QPBox;
        enableFullDDP = fullDDP;

        if(enableQPBox) TRACE("Box QP is enabled\n");
        else TRACE("Box QP is disabled\n");


        xList.resize(stateSize, N + 1);
        uList.resize(commandSize, N);

        uListFull.resize(commandSize, N + 1);
        updatedxList.resize(stateSize, N + 1);
        updateduList.resize(commandSize, N);
        costList.resize(N + 1);
        costListNew.resize(N + 1);

        kList.resize(commandSize, N);
        KList.resize(N);
        FList.resize(stateSize, N + 1);
        Vx.resize(stateSize, N + 1);
        Vxx.resize(N + 1);
        
        xList.setZero();
        uList.setZero();
        uListFull.setZero();
        updatedxList.setZero();
        updateduList.setZero();
        kList.setZero();
        FList.setZero(); 
        Vx.setZero();

        for (unsigned int i = 0; i < N; i++)
        {
            costList[i] = 0;
            costListNew[i] = 0;
            KList[i].setZero();
            Vxx[i].setZero();
        }

        costList[N] = 0;
        costListNew[N] = 0;
        Vxx[N].setZero();
        
        k.setZero();
        K.setZero();
        dV.setZero();

        // parameters for line search
        Op.alphaList.resize(2);
        Op.alphaList << 1.0, 0.5012;//, 0.2512, 0.1259;//, 0.0631, 0.0316, 0.0158, 0.0079, 0.0040, 0.0020, 0.0010;

        debugging_print = 0;

        u_NAN_loc(0) = sqrt(-1.0);
    }


    void solve(const stateVec_t& x_0, const commandVecTab_t& u_0, const stateVecTab_t &x_track, const Eigen::MatrixXd& cList_bar, 
               const stateVecTab_t& xList_bar, const commandVecTab_t& uList_bar, const Eigen::MatrixXd& thetaList_bar, const Eigen::VectorXd& rho, const Eigen::VectorXd& R_c)
    {

        if(Op.debug_level > 0) {TRACE("begin iterative LQR...");}
        initializeTrajectory(x_0, u_0, x_track, cList_bar, xList_bar, uList_bar, thetaList_bar, rho, R_c);

        Op.lambda = Op.lambdaInit;
        Op.dlambda = Op.dlambdaInit;
        
        for (iter = 0; iter < Op.max_iter; iter++)
        {

            if (newDeriv)
            {
                for (unsigned int i = 0; i < commandSize; i++)
                {
                    u_NAN(i) = sqrt(-1.0); // control vector = Nan for last time step
                }
                
                for (unsigned int i = 0; i < uList.cols(); i++) 
                {
                    uListFull.col(i) = uList.col(i);
                }
                uListFull.rightCols(1) = u_NAN;

                /* ---------------- forwad pass ----------------------- */
                /* -------------- compute fx, fu ---------------------- */
                dynamicModel->fx(xList, uListFull);
                
                /* -------------- compute cx, cu, cxx, cuu ------------ */
                costFunction->computeDerivatives(xList, uListFull, x_track, cList_bar, xList_bar, uList_bar, thetaList_bar, rho, R_c);
                newDeriv = 0;
            }


            // STEP 2: backward pass, compute optimal control law and cost-to-go;
            backPassDone = 0;
            while (!backPassDone)
            {

                doBackwardPass();
                if (diverge)
                {
                    if (Op.debug_level > 1) printf("Cholesky failed at timestep %d.\n", diverge);
                    Op.dlambda   = max(Op.dlambda * Op.lambdaFactor, Op.lambdaFactor);
                    Op.lambda    = max(Op.lambda * Op.dlambda, Op.lambdaMin);
                    if (Op.lambda > Op.lambdaMax) { break; };
                }

                backPassDone = 1;
            }

            // check for termination due to small gradient
            // TODO: add constraint tolerance check
            if (Op.g_norm < Op.tolGrad && Op.lambda < 1e-5)
            {
                Op.dlambda = min(Op.dlambda / Op.lambdaFactor, 1.0/Op.lambdaFactor);
                Op.lambda  = Op.lambda * Op.dlambda * (Op.lambda > Op.lambdaMin);
                if (Op.debug_level >= 1) {
                    TRACE(("\nSUCCESS: gradient norm < tolGrad\n"));
                }
                break;
            }

            // STEP 3: line-search to find new control sequence, trajectory, cost
            fwdPassDone = 0;
            if (backPassDone)
            {
                // only implement serial backtracking line-search
                for (int alpha_index = 0; alpha_index < Op.alphaList.size(); alpha_index++)
                {
                    alpha = Op.alphaList[alpha_index];

                    doForwardPass(x_0, x_track, cList_bar, xList_bar, uList_bar, thetaList_bar, rho, R_c);
                    Op.dcost = accumulate(costList.begin(), costList.end(), 0.0) - accumulate(costListNew.begin(), costListNew.end(), 0.0);
                    Op.expected = -alpha*(dV(0) + alpha*dV(1));

                    double z;
                    if (Op.expected > 0) {
                        z = Op.dcost / Op.expected;
                    } else {
                        z = static_cast<double>(-signbit(Op.dcost)); // [TODO:doublecheck]
                        TRACE("non-positive expected reduction: should not occur \n"); 
                    }

                    if(z > Op.zMin) { 
                        fwdPassDone = 1;
                        break;
                    }
                }
                if(!fwdPassDone) alpha = sqrt(-1.0);
            }
                    
            // STEP 4: accept step (or not), draw graphics, print status
            if (Op.debug_level > 1 && Op.last_head == Op.print_head)
            {
                Op.last_head = 0;
                TRACE("iteration,\t cost, \t reduction, \t expected, \t gradient, \t log10(lambda) \n");
            }
            
            if (fwdPassDone)
            {
                // print status
                if (Op.debug_level > 1)
                {
                    if(!debugging_print) printf("%-14d%-12.6g%-15.3g%-15.3g%-19.3g%-17.1f\n", iter + 1, accumulate(costList.begin(), costList.end(), 0.0), Op.dcost, Op.expected, Op.g_norm, log10(Op.lambda));
                    Op.last_head = Op.last_head + 1;
                }

                Op.dlambda = min(Op.dlambda / Op.lambdaFactor, 1.0/Op.lambdaFactor);
                Op.lambda  = Op.lambda * Op.dlambda * (Op.lambda > Op.lambdaMin);

                // accept changes
                xList = updatedxList;
                uList = updateduList;
                costList = costListNew;
                newDeriv = 1;

                // TODO: add constraint tolerance check
                if(Op.dcost < Op.tolFun) 
                {
                    if(Op.debug_level >= 1) 
                    {
                        TRACE(("\nSUCCESS: cost change < tolFun\n"));
                    }
                
                    break;
                }
            }
            else {   
                // no cost improvement
                // increase lambda
                Op.dlambda = max(Op.dlambda * Op.lambdaFactor, Op.lambdaFactor);
                Op.lambda  = max(Op.lambda * Op.dlambda, Op.lambdaMin);

                // print status
                if (Op.debug_level >= 1)
                {
                    if(!debugging_print) printf("%-14d%-12.9s%-15.3g%-15.3g%-19.3g%-17.1f\n", iter+1, "No STEP", Op.dcost, Op.expected, Op.g_norm, log10(Op.lambda));
                    Op.last_head = Op.last_head+1;
                }

                if (Op.lambda > Op.lambdaMax) 
                {
                    if(Op.debug_level >= 1)
                        TRACE(("\nEXIT: lambda > lambdaMax\n"));
                    break;
                }
            }
        }

        Op.iterations = iter;

        if(!backPassDone) {
            if(Op.debug_level >= 1)
                TRACE(("\nEXIT: no descent direction found.\n"));
            
            return;    
        } else if(iter >= Op.max_iter) {
            if(Op.debug_level >= 1)
                TRACE(("\nEXIT: Maximum iterations reached.\n"));
            
            return;
        }
    }

    void initializeTrajectory(const State& x_0, const ControlTrajectory& u_0, const stateVecTab_t &x_track, const Eigen::MatrixXd& cList_bar, const StateTrajectory& xList_bar,
    const ControlTrajectory& uList_bar, const Eigen::MatrixXd& thetaList_bar, const Eigen::VectorXd& rho, const Eigen::VectorXd& R_c)
    {
        xList.col(0) = x_0;

        initFwdPassDone = 0;
        diverge = 1;
        
        uList = std::move(u_0);

        updatedxList.col(0) = x_0;

        isUNan = 0;

        for (int i = 0; i < N; i++) 
        {
            costList[i]             = costFunction->cost_func_expre_admm(i, updatedxList.col(i), u_0.col(i), x_track.col(i), cList_bar.col(i), xList_bar.col(i), uList_bar.col(i), thetaList_bar.col(i), rho, R_c);
            updatedxList.col(i + 1) = forward_integration(updatedxList.col(i), u_0.col(i));
        }


        costList[N]  = costFunction->cost_func_expre_admm(N, updatedxList.col(N), u_NAN_loc, x_track.col(N), cList_bar.col(N), xList_bar.col(N), u_NAN_loc, thetaList_bar.col(N), rho, R_c);


        /* simplistic divergence test, check for the last time step if it has diverged. */
        int diverge_element_flag = 0;
        for (int j = 0; j < stateSize; j++)
        {
            if (fabs(xList(j, N - 1)) > 1e8)
            { 
                diverge_element_flag = 1; // checking the absolute value
            }
        }
        
        initFwdPassDone = 1;
        xList = updatedxList;

        newDeriv      = 1; 
        Op.lambda     = Op.lambdaInit;
        Op.dcost      = 0;
        Op.expected   = 0;
        Op.print_head = 6;
        Op.last_head  = Op.print_head;

    }

    const struct traj& getLastSolvedTrajectory()
    {
        lastTraj.xList       = xList;
        lastTraj.uList       = uList;
        lastTraj.KList       = KList;
        lastTraj.kList       = kList;
        lastTraj.finalCost   = accumulate(costList.begin(), costList.end(), 0.0);
        lastTraj.finalGrad   = Op.g_norm;
        lastTraj.finalLambda = log10(Op.lambda);
        return lastTraj;
    }

    const ControlStateGains& getStateGains() const
    {
        return KList;
    }


private:
    inline State forward_integration(const State& x, const Control& u)
    {
        x_dot1 = dynamicModel->f(x, u);
        x_dot2 = dynamicModel->f(x + 0.5 * dt * x_dot1, u);
        x_dot3 = dynamicModel->f(x + 0.5 * dt * x_dot2, u);
        x_dot4 = dynamicModel->f(x + dt * x_dot3, u);

        return x + (dt/6) * (x_dot1 + 2 * x_dot2 + 2 * x_dot3 + x_dot4);
    }

    void doBackwardPass()
    {    
        // if (Op.regType == 1) {
        //     lambdaEye = Op.lambda * stateMat_t::Identity();
        // } else {
        //     lambdaEye = Op.lambda * stateMat_t::Zero();
        // }

        diverge = 0;
        
        g_norm_sum = 0.0;
        Vx.col(N)  = costFunction->getcx().col(N);
        Vxx[N]     = costFunction->getcxx()[N];
        dV.setZero();

        for (int i = N-1; i >= 0; i--) {

            Qx  = costFunction->getcx().col(i)  + dynamicModel->getfxList()[i].transpose() *  Vx.col(i + 1);
            Qu  = costFunction->getcu().col(i)  + dynamicModel->getfuList()[i].transpose() *  Vx.col(i + 1);
            Qxx = costFunction->getcxx()[i]     + dynamicModel->getfxList()[i].transpose() * Vxx[i + 1]  * dynamicModel->getfxList()[i];
            Quu = costFunction->getcuu()[i]     + dynamicModel->getfuList()[i].transpose() * Vxx[i + 1]  * dynamicModel->getfuList()[i];
            Qux = costFunction->getcux()[i]     + dynamicModel->getfuList()[i].transpose() * Vxx[i + 1]  * dynamicModel->getfxList()[i];

            if (Op.regType == 1) 
            {
                QuuF = Quu + Op.lambda * commandMat_t::Identity();
            } else {
                QuuF = Quu;
            }
            
            QuuInv = QuuF.inverse();

            if (!isPositiveDefinite(Quu)) {
                // To be Implemented : Regularization (is Quu definite positive ?)
                TRACE("Quu is not positive definite ");
                if (Op.lambda==0.0) {
                    Op.lambda += 1e-4;
                } else {   
                    Op.lambda *= 10;
                }
                backPassDone = 0;
                break;
            }

            if (!enableQPBox)
            {
                // Cholesky decomposition by using upper triangular matrix
                Eigen::LLT<MatrixXd> lltOfQuuF(QuuF);
                Eigen::MatrixXd L = lltOfQuuF.matrixU(); 
                // assume QuuF is positive definite
                
                // A temporary solution: check the non-PD case. TODO: double check here.
                if (lltOfQuuF.info() == Eigen::NumericalIssue)
                {
                    diverge = i;
                    TRACE("Possibly non semi-positive definitie matrix!");
                    return;
                }

                Eigen::MatrixXd L_inverse = L.inverse();
                k = - L_inverse * L.transpose().inverse() * Qu;
                K = - L_inverse * L.transpose().inverse() * Qux;
            }

            // update cost-to-go approximation
            dV(0) += k.transpose() * Qu;

            dV(1) += 0.5 * k.transpose() * Quu * k;

            Vx.col(i)    = Qx  + K.transpose() * Quu * k + K.transpose() * Qu  + Qux.transpose() * k;
            Vxx[i]       = Qxx + K.transpose() * Quu * K + K.transpose() * Qux + Qux.transpose() * K;
            Vxx[i]       = 0.5 * (Vxx[i] + Vxx[i].transpose());

            kList.col(i) = k;
            KList[i]     = K;


            g_norm_max= 0.0;

            for (int j = 0; j < commandSize; j++) 
            {
                g_norm_i = fabs(kList.col(i)(j)) / (fabs(uList.col(i)(j)) + 1.0);
                if(g_norm_i > g_norm_max) g_norm_max = g_norm_i;
            }
            g_norm_sum += g_norm_max;
        }

        Op.g_norm = g_norm_sum / (static_cast<double>(Op.n_hor));
    }

    void doForwardPass(const stateVec_t& x_0, const stateVecTab_t &x_track, const Eigen::MatrixXd& cList_bar, const stateVecTab_t& xList_bar, const commandVecTab_t& uList_bar, 
        const Eigen::MatrixXd& thetaList_bar, const Eigen::VectorXd& rho, const Eigen::VectorXd& R_c)
    {
        updatedxList.col(0) = x_0;

        for (unsigned int i = 0; i < N; i++) 
        {
            updateduList.col(i)     = uList.col(i) + alpha * kList.col(i) + KList[i] * (updatedxList.col(i) - xList.col(i));
            costListNew[i]          = costFunction->cost_func_expre_admm(i, updatedxList.col(i), updateduList.col(i), x_track.col(i), 
                                                                            cList_bar.col(i), xList_bar.col(i), uList_bar.col(i), thetaList_bar.col(i), rho, R_c);
            updatedxList.col(i + 1) = forward_integration(updatedxList.col(i), updateduList.col(i));

        }

        costListNew[N] = costFunction->cost_func_expre_admm(N, updatedxList.col(N), u_NAN_loc, x_track.col(N), cList_bar.col(N), xList_bar.col(N), uList_bar.col(N-1), thetaList_bar.col(N-1), rho, R_c);
    }

    inline bool isPositiveDefinite(const ControlHessian& Quu_p)
    {

        Eigen::LLT<Eigen::MatrixXd> lltOfA(Quu_p); // compute the Cholesky decomposition of A
        if(lltOfA.info() == Eigen::NumericalIssue)
        {
            // throw std::runtime_error("Possibly non semi-positive definitie matrix!");
            TRACE("Matrix is not PSD");
            return false;
        }  

        return true;
    }
};


} // namesapce

#endif 

