#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <sys/time.h>

#define ENABLE_QPBOX 0
#define DISABLE_QPBOX 1
#define ENABLE_FULLDDP 0
#define DISABLE_FULLDDP 1

#define SOFT_CONTACT
#define CONTACT_EN 1
#define DEBUG 


const int stateSize = 17;
const int commandSize = 7;
const int fullstatecommandSize = 24;
const int NDOF = 7;

const double TimeHorizon = 30;
const double TimeStep = 0.01; // 0.01s works for soft contact dynamics

const unsigned int NumberofKnotPt = TimeHorizon / TimeStep;
const int32_t kNumJoints = 7;



// typedef for stateSize types
typedef Eigen::Matrix<double, stateSize, 1> stateVec_t;                       // stateSize x 1
typedef Eigen::Matrix<double, 1, stateSize> stateVecTrans_t;                  // 1 x stateSize
typedef Eigen::Matrix<double, stateSize, stateSize> stateMat_t;               // stateSize x stateSize
typedef Eigen::Matrix<double, stateSize, stateSize> stateTens_t[stateSize];   // stateSize x stateSize x stateSize

// typedef for commandSize types
typedef Eigen::Matrix<double,commandSize,1> commandVec_t;                           // commandSize x 1
typedef Eigen::Matrix<double,1,commandSize> commandVecTrans_t;                      // 1 x commandSize
typedef Eigen::Matrix<double,commandSize,commandSize> commandMat_t;                 // commandSize x commandSize
typedef Eigen::Matrix<double,commandSize,commandSize> commandTens_t[commandSize];   // stateSize x commandSize x commandSize

// typedef for mixed stateSize and commandSize types
typedef Eigen::Matrix<double, stateSize, commandSize> stateR_commandC_t;                          // stateSize x commandSize
typedef Eigen::Matrix<double, stateSize, commandSize> stateR_commandC_stateD_t[stateSize];        // stateSize x commandSize x stateSize
typedef Eigen::Matrix<double, stateSize, commandSize> stateR_commandC_commandD_t[commandSize];    // stateSize x commandSize x commandSize
typedef Eigen::Matrix<double, commandSize, stateSize> commandR_stateC_t;                          // commandSize x stateSize
typedef Eigen::Matrix<double, commandSize, stateSize> commandR_stateC_stateD_t[stateSize];        // commandSize x stateSize x stateSize
typedef Eigen::Matrix<double, commandSize, stateSize> commandR_stateC_commandD_t[commandSize];    // commandSize x stateSize x commandSize
typedef Eigen::Matrix<double, stateSize, stateSize> stateR_stateC_commandD_t[commandSize];        // stateSize x stateSize x commandSize
typedef Eigen::Matrix<double, commandSize, commandSize> commandR_commandC_stateD_t[stateSize];    // commandSize x commandSize x stateSize
typedef Eigen::Matrix<double, stateSize + commandSize, 1> stateAug_t;                               // stateSize + commandSize x 1
typedef Eigen::Matrix<double, stateSize + commandSize, 1> projStateAndCommand_t;                    // 21 x 1
typedef double scalar_t;

typedef Eigen::Matrix<double, NDOF, 1> stateVec_half_t;                      // stateSize/2 x 1
typedef Eigen::Matrix<double, NDOF, NDOF> stateMat_half_t;                   // stateSize/2 x stateSize/2
typedef Eigen::Matrix<double, NDOF, commandSize> stateR_half_commandC_t;     // stateSize/2 x commandSize


/* --------------------------------------------------------------------------------------------------------------------------- */
typedef Eigen::Matrix<double, stateSize, Eigen::Dynamic> stateVecTab_t;
typedef std::vector<double> costVecTab_t;
typedef Eigen::Matrix<double, commandSize, Eigen::Dynamic> commandVecTab_t;

typedef std::vector<stateMat_t, Eigen::aligned_allocator<stateMat_t> > stateMatTab_t;
typedef std::vector<commandMat_t, Eigen::aligned_allocator<commandMat_t> > commandMatTab_t;

typedef std::vector<stateR_commandC_t, Eigen::aligned_allocator<stateR_commandC_t> > stateR_commandC_tab_t;
typedef std::vector<commandR_stateC_t, Eigen::aligned_allocator<commandR_stateC_t> > commandR_stateC_tab_t;

typedef Eigen::Matrix<double, (stateSize-3)/2, Eigen::Dynamic> stateVecTab_half_t;
typedef Eigen::Matrix<double, stateSize + commandSize, Eigen::Dynamic> projStateAndCommandTab_t;

typedef std::vector<std::vector<stateMat_t, Eigen::aligned_allocator<stateMat_t> > > stateTensTab_t;
typedef std::vector<std::vector<stateR_commandC_t, Eigen::aligned_allocator<stateR_commandC_t> > > stateR_commandC_Tens_t;


