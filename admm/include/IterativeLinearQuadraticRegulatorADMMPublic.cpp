#pragma once


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