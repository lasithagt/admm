#include "ilqrsolver.h"

/* Debug */
#include <iostream>
using namespace std;
/* */

using namespace Eigen;
using Eigen::VectorXd;

namespace optimizer {

ILQRSolver::ILQRSolver(RobotDynamics& iiwaDynamicModel, CostFunction& iiwaCostFunction, const OptSet& solverOptions, const int& time_steps, const double& dt_, bool fullDDP, bool QPBox) : 
        N(time_steps), dt(dt_), Op(solverOptions)
{
    // TRACE("initialize dynamic model and cost function\n");
    // TODO : could be optimized here

    dynamicModel  = &iiwaDynamicModel;
    costFunction  = &iiwaCostFunction;

    enableQPBox   = QPBox;
    enableFullDDP = fullDDP;

    if(enableQPBox) TRACE("Box QP is enabled\n");
    else TRACE("Box QP is disabled\n");

    if(enableFullDDP) TRACE("Full DDP is enabled\n");
    else TRACE("Full DDP is disabled\n");


    Op.time_backward.resize(Op.max_iter);
    Op.time_backward.setZero();
    Op.time_forward.resize(Op.max_iter);
    Op.time_forward.setZero();
    Op.time_derivative.resize(Op.max_iter);
    Op.time_derivative.setZero();

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
    Op.alphaList.resize(11);
    Op.alphaList << 1.0, 0.5012, 0.2512, 0.1259, 0.0631, 0.0316, 0.0158, 0.0079, 0.0040, 0.0020, 0.0010;

    debugging_print = 0;


}


/**
 * @brief               Instantiate the receding horizon wrapper for the trajectory optimizer.
 * @param dt            Time step
 * @param time_steps    Number of time steps over which to optimize
 * @param iterations    The number of iterations to perform per time step
 * @param logger        util::Logger to use for informational, warning, and error messages
 * @param verbose       True if informational and warning messages should be passed to the logger; error messages are always passed
 * @param args          Arbitrary arguments to pass to the trajectory optimizer at initialization time
 */
void ILQRSolver::solve(const stateVec_t& x_0, const commandVecTab_t& u_0, const stateVecTab_t &xtrack)
{
    //==============
    // Checked!!v
    //==============

    initializeTraj(x_0, u_0, xtrack);


    Op.lambda = Op.lambdaInit;
    Op.dlambda = Op.dlambdaInit;
    
    // TODO: update multipliers 

    for (iter = 0; iter < Op.max_iter; iter++)
    {

        // differentiate dynamics and cost along nominal trajectory
        if (newDeriv) {
            for (unsigned int i = 0; i < commandSize; i++) {
                u_NAN(i) = sqrt(-1.0); // control vector = Nan for last time step
            }
            
            for (unsigned int i = 0; i < uList.cols(); i++) {
                uListFull.col(i) = uList.col(i);
            }
            uListFull.rightCols(1) = u_NAN;

            gettimeofday(&tbegin_time_deriv,NULL);

            /* ---------------- forwad pass ----------------------- */

            /* -------------- compute fx, fu ---------------------- */
            dynamicModel->fx(xList, uListFull);
            

            /* -------------- compute cx, cu, cxx, cuu ------------ */
            costFunction->computeDerivatives(xList, uListFull, xtrack);

            gettimeofday(&tend_time_deriv,NULL);
            Op.time_derivative(iter) = (static_cast<double>(1000*(tend_time_deriv.tv_sec-tbegin_time_deriv.tv_sec)+((tend_time_deriv.tv_usec-tbegin_time_deriv.tv_usec)/1000)))/1000.0;

            newDeriv = 0;
        }


        //  backward pass, compute optimal control law and cost-to-go
        backPassDone = 0;
        while (!backPassDone)
        {
            gettimeofday(&tbegin_time_bwd,NULL);

            doBackwardPass();

            gettimeofday(&tend_time_bwd,NULL);
            Op.time_backward(iter) = (static_cast<double>(1000*(tend_time_bwd.tv_sec-tbegin_time_bwd.tv_sec)+((tend_time_bwd.tv_usec-tbegin_time_bwd.tv_usec)/1000)))/1000.0;

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
            Op.lambda = Op.lambda * Op.dlambda * (Op.lambda > Op.lambdaMin);
            if (Op.debug_level >= 1)
            {
                TRACE(("\nSUCCESS: gradient norm < tolGrad\n"));
            }
            break;
        }

        //====== STEP 3: line-search to find new control sequence, trajectory, cost
        fwdPassDone = 0;
        if (backPassDone) {
            gettimeofday(&tbegin_time_fwd,NULL);
            //only implement serial backtracking line-search
            for (int alpha_index = 0; alpha_index < Op.alphaList.size(); alpha_index++) {

                alpha = Op.alphaList[alpha_index];
                doForwardPass(x_0, xtrack);
                Op.dcost = accumulate(costList.begin(), costList.end(), 0.0) - accumulate(costListNew.begin(), costListNew.end(), 0.0);
                Op.expected = -alpha*(dV(0) + alpha*dV(1));


                double z;
                if (Op.expected > 0) {
                    z = Op.dcost / Op.expected;
                } else {
                    z = static_cast<double>(-signbit(Op.dcost)); //[TODO:doublecheck]
                    TRACE("non-positive expected reduction: should not occur \n"); //warning
                }

                if(z > Op.zMin) { 
                    fwdPassDone = 1;
                    break;
                }
            }
            if(!fwdPassDone) alpha = sqrt(-1.0);
            gettimeofday(&tend_time_fwd,NULL);
            Op.time_forward(iter) = (static_cast<double>(1000*(tend_time_fwd.tv_sec-tbegin_time_fwd.tv_sec)+((tend_time_fwd.tv_usec-tbegin_time_fwd.tv_usec)/1000)))/1000.0;
        }
        
        //====== STEP 4: accept step (or not), draw graphics, print status
        if (Op.debug_level > 1 && Op.last_head == Op.print_head) {
            Op.last_head = 0;
            TRACE("iteration,\t cost, \t reduction, \t expected, \t gradient, \t log10(lambda) \n");
        }
        
        if (fwdPassDone) {
            if (Op.debug_level > 1)
            {
                if(!debugging_print) printf("%-14d%-12.6g%-15.3g%-15.3g%-19.3g%-17.1f\n", iter+1, accumulate(costList.begin(), costList.end(), 0.0), Op.dcost, Op.expected, Op.g_norm, log10(Op.lambda));
                Op.last_head = Op.last_head + 1;
            }

            Op.dlambda = min(Op.dlambda / Op.lambdaFactor, 1.0/Op.lambdaFactor);
            Op.lambda  = Op.lambda * Op.dlambda * (Op.lambda > Op.lambdaMin);

            // accept changes
            xList = updatedxList;
            uList = updateduList;
            costList = costListNew;
            newDeriv = 1;

            // terminate ?
            // TODO: add constraint tolerance check
            if(Op.dcost < Op.tolFun) {
                if(Op.debug_level >= 1) {
                    TRACE(("\n SUCCESS: cost change < tolFun \n"));
                }
            
                break;
            }
        } else { 

            // if no cost improvement, increase lambda
            Op.dlambda= max(Op.dlambda * Op.lambdaFactor, Op.lambdaFactor);
            Op.lambda= max(Op.lambda * Op.dlambda, Op.lambdaMin);

            // print status
            if(Op.debug_level >= 1) {
                if(!debugging_print) printf("%-14d%-12.9s%-15.3g%-15.3g%-19.3g%-17.1f\n", iter+1, "No STEP", Op.dcost, Op.expected, Op.g_norm, log10(Op.lambda));
                Op.last_head = Op.last_head+1;
            }

            // terminate ?
            if(Op.lambda > Op.lambdaMax) {
                if(Op.debug_level >= 1)
                    TRACE(("\nEXIT: lambda > lambdaMax\n"));
                break;
            }
        }
    }

    Op.iterations = iter;

    if (!backPassDone) {
        if(Op.debug_level >= 1)
            TRACE(("\nEXIT: no descent direction found.\n"));
        
        return;    
    } else if(iter >= Op.max_iter) {
        if(Op.debug_level >= 1)
            TRACE(("\nEXIT: Maximum iterations reached.\n"));
        
        return;
    }
}

/**
 * @brief               Instantiate the receding horizon wrapper for the trajectory optimizer.
 * @param dt            Time step
 * @param time_steps    Number of time steps over which to optimize
 * @param iterations    The number of iterations to perform per time step
 * @param logger        util::Logger to use for informational, warning, and error messages
 * @param verbose       True if informational and warning messages should be passed to the logger; error messages are always passed
 * @param args          Arbitrary arguments to pass to the trajectory optimizer at initialization time
 */
void ILQRSolver::initializeTraj(const stateVec_t& x_0, const commandVecTab_t& u_0, const stateVecTab_t &x_track)
{
    xList.col(0) = x_0;
    commandVec_t zeroCommand;
    zeroCommand.setZero();

    // (low priority) TODO: implement control limit selection
    // (low priority) TODO: initialize trace data structure

    initFwdPassDone = 0;
    diverge = 1;
    
    for (int i = 0; i < N; i++) {
        uList.col(i) = u_0.col(i);
    }

    updatedxList.col(0) = x_0;

    commandVec_t u_NAN_loc;
    u_NAN_loc(0) = sqrt(-1.0);
    isUNan = 0;

    for (unsigned int i = 0; i < N; i++) {
        updateduList.col(i)     = uList.col(i);
        costList[i]             = costFunction->cost_func_expre(i, updatedxList.col(i), updateduList.col(i), x_track.col(i));
        updatedxList.col(i + 1) = forward_integration(updatedxList.col(i), updateduList.col(i));

    }
    // getting final cost, state, input=NaN

    costList[N] = costFunction->cost_func_expre(N, updatedxList.col(N), u_NAN_loc, x_track.col(N));


    // simplistic divergence test, check for the last time step if it has diverged.
    int diverge_element_flag = 0;
    for (unsigned int j = 0; j < stateSize; j++)
    {
        if (fabs(xList(j, N - 1)) > 1e8)
        { 
            diverge_element_flag = 1; // checking the absolute value
        }
    }
    
    initFwdPassDone = 1;
    xList = updatedxList;

    //constants, timers, counters
    newDeriv = 1; // flgChange
    Op.lambda= Op.lambdaInit;
    Op.dcost = 0;
    Op.expected = 0;
    Op.print_head = 6;
    Op.last_head = Op.print_head;


    if(Op.debug_level > 0) {TRACE("\n =========== begin iLQR =========== \n");}
}

/**
 * @brief               Instantiate the receding horizon wrapper for the trajectory optimizer.
 * @param dt            Time step
 * @param time_steps    Number of time steps over which to optimize
 * @param iterations    The number of iterations to perform per time step
 * @param logger        util::Logger to use for informational, warning, and error messages
 * @param verbose       True if informational and warning messages should be passed to the logger; error messages are always passed
 * @param args          Arbitrary arguments to pass to the trajectory optimizer at initialization time
 */
void ILQRSolver::doForwardPass(const stateVec_t& x_0, const stateVecTab_t &x_track)
{

    updatedxList.col(0) = x_0;

    commandVec_t u_NAN_loc;
    u_NAN_loc(0) = sqrt(-1.0);

    isUNan = 0;

    for (unsigned int i = 0; i < N; i++) 
    {
        updateduList.col(i)     = uList.col(i) + alpha * kList.col(i) + KList[i] * (updatedxList.col(i) - xList.col(i));
        costListNew[i]          = costFunction->cost_func_expre(i, updatedxList.col(i), updateduList.col(i), x_track.col(i));
        updatedxList.col(i + 1) = forward_integration(updatedxList.col(i), updateduList.col(i));
    }

    costListNew[N] = costFunction->cost_func_expre(N, updatedxList.col(N), u_NAN_loc, x_track.col(N));
}

/* 4th-order Runge-Kutta step */
inline stateVec_t ILQRSolver::forward_integration(const stateVec_t& x, const commandVec_t& u)
{
    // if(debugging_print) TRACE_KUKA_ARM("update: 4th-order Runge-Kutta step\n");

    // gettimeofday(&tbegin_period4, NULL);

    stateVec_t x_dot1 = dynamicModel->f(x, u);
    stateVec_t x_dot2 = dynamicModel->f(x + 0.5 * dt * x_dot1, u);
    stateVec_t x_dot3 = dynamicModel->f(x + 0.5 * dt * x_dot2, u);
    stateVec_t x_dot4 = dynamicModel->f(x + dt * x_dot3, u);

    stateVec_t x_new;
    x_new = x + (dt/6) * (x_dot1 + 2 * x_dot2 + 2 * x_dot3 + x_dot4);

    return x_new;
}

void ILQRSolver::doBackwardPass()
{    
    if (Op.regType == 1)
    {
        lambdaEye = Op.lambda * stateMat_t::Identity();
    }
    else
    {
        lambdaEye = Op.lambda * stateMat_t::Zero();
    }

    diverge = 0;
    
    g_norm_sum = 0.0;
    Vx.col(N)  = costFunction->getcx().col(N);
    Vxx[N]     = costFunction->getcxx()[N];
    dV.setZero();

    for (int i = static_cast<int>(N-1); i >= 0; i--)
    {
        Qx  = costFunction->getcx().col(i)  + dynamicModel->getfxList()[i].transpose() * Vx.col(i + 1);
        Qu  = costFunction->getcu().col(i)  + dynamicModel->getfuList()[i].transpose() * Vx.col(i + 1);
        Qxx = costFunction->getcxx()[i]     + dynamicModel->getfxList()[i].transpose() * Vxx[i + 1]  * dynamicModel->getfxList()[i];
        Quu = costFunction->getcuu()[i]     + dynamicModel->getfuList()[i].transpose() * Vxx[i + 1]  * dynamicModel->getfuList()[i];
        Qux = costFunction->getcux()[i]     + dynamicModel->getfuList()[i].transpose() * Vxx[i + 1]  * dynamicModel->getfxList()[i];
 

        if (Op.regType == 1)
        {
            QuuF = Quu + Op.lambda * commandMat_t::Identity();
        }
        else
        {
            QuuF = Quu;
        }
        
        QuuInv = QuuF.inverse();

        if (!isPositiveDefinite(Quu))
        {
            //To be Implemented : Regularization (is Quu definite positive ?)
            TRACE("Quu is not positive definite ");
            if (Op.lambda==0.0) 
            {
                Op.lambda += 1e-4;
            }
            else 
            {   
                Op.lambda *= 10;
            }
            backPassDone = 0;
            break;
        }

        // if(enableQPBox)
        // {
        //     //TRACE("Use Box QP");
        //     nWSR = 10; //[to be checked]
        //     H = Quu;
        //     g = Qu;
        //     lb = lowerCommandBounds - uList[i];
        //     ub = upperCommandBounds - uList[i];
        //     qp->init(H.data(),g.data(),lb.data(),ub.data(),nWSR);
        //     qp->getPrimalSolution(xOpt);
        //     k = Map<commandVec_t>(xOpt);
        //     K = -QuuInv*Qux;
        //     for(unsigned int i_cmd=0;i_cmd<commandNb;i_cmd++)
        //     {
        //         if((k[i_cmd] == lowerCommandBounds[i_cmd]) | (k[i_cmd] == upperCommandBounds[i_cmd]))
        //         {
        //             K.row(i_cmd).setZero();
        //         }
        //     }
        // }

        if (!enableQPBox)
        {
            // Cholesky decomposition by using upper triangular matrix
            Eigen::LLT<MatrixXd> lltOfQuuF(QuuF);
            Eigen::MatrixXd L = lltOfQuuF.matrixU(); 
            // assume QuuF is positive definite
            
            // A temporary solution: check the non-PD case
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

        Vx.col(i)  = Qx  + K.transpose() * Quu * k + K.transpose() * Qu  + Qux.transpose() * k;
        Vxx[i] = Qxx + K.transpose() * Quu * K + K.transpose() * Qux + Qux.transpose() * K;
        Vxx[i] = 0.5 * (Vxx[i] + Vxx[i].transpose());

        kList.col(i) = k;
        KList[i] = K;

        g_norm_max= 0.0;

        for (unsigned int j = 0; j < commandSize; j++) 
        {
            g_norm_i = fabs(kList.col(i)(j)) / (fabs(uList.col(i)(j)) + 1.0);
            if(g_norm_i > g_norm_max) g_norm_max = g_norm_i;
        }
        g_norm_sum += g_norm_max;
    }
    Op.g_norm = g_norm_sum / (static_cast<double>(Op.n_hor));
}



ILQRSolver::traj ILQRSolver::getLastSolvedTrajectory()
{
    lastTraj.xList = xList;
    lastTraj.uList = uList;
    lastTraj.iter = iter;
    lastTraj.finalCost = accumulate(costList.begin(), costList.end(), 0.0);
    lastTraj.finalGrad = Op.g_norm;
    lastTraj.finalLambda = log10(Op.lambda);
    lastTraj.time_forward = Op.time_forward;
    lastTraj.time_backward = Op.time_backward;
    lastTraj.time_derivative = Op.time_derivative;
    return lastTraj;
}

bool ILQRSolver::isPositiveDefinite(const commandMat_t & Quu_p)
{
    //Eigen::JacobiSVD<commandMat_t> svd_Quu (Quu, ComputeThinU | ComputeThinV);
    Eigen::VectorXcd singular_values = Quu_p.eigenvalues();

    for(long i = 0; i < Quu_p.cols(); ++i)
    {
        if (singular_values[i].real() < 0.)
        {
            TRACE("Matrix is not SDP");
            return false;
        }
    }
    return true;
}

}
