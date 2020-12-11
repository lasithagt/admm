#pragma once

#ifndef CONFIG_H
#define CONFIG_H

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <numeric>
#include <sys/time.h>
#include <cstdarg>

#define ENABLE_QPBOX 0
#define DISABLE_QPBOX 1
#define ENABLE_FULLDDP 0
#define DISABLE_FULLDDP 1

#define WHOLE_BODY 1
#define CARTESIAN_FRAME 10

#define SOFT_CONTACT 1
#define CONTACT_EN 1
#define NUM_COST 1
#define DEBUG 1
#define TRACK 1 // for tracking vs goal

#define MULTI_THREAD 0
#if MULTI_THREAD
#include <thread>
#define NUMBER_OF_THREAD 1 
#endif

#define stateSize 17 
#define commandSize 7 
#define fullstatecommandSize 24
#define NDOF 7

#define TimeHorizon 10
#define TimeStep 0.01 // 0.01s works for soft contact dynamics

#define NumberofKnotPt TimeHorizon / TimeStep
#define InterpolationScale 10 
const int32_t kNumJoints = 7;


#define BOXWEIGHT 0.122


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


class Logger
{
public:
    Logger() = default;
    Logger(const Logger &other) = default;
    Logger(Logger &&other) = default;
    virtual ~Logger() = default;
    Logger& operator=(const Logger &other) = default;
    Logger& operator=(Logger &&other) = default;

    /**
     * @brief       Pure virtual function to log informational messages.
     * @param fmt   Format string (if additional arguments are passed) or message to display
     */
    virtual void info(const char *fmt, ...) = 0;
    /**
     * @brief       Pure virtual function to log warnings.
     * @param fmt   Format string (if additional arguments are passed) or message to display
     */
    virtual void warning(const char *fmt, ...) = 0;
    /**
     * @brief       Pure virtual function to log errors.
     * @param fmt   Format string (if additional arguments are passed) or message to display
     */
    virtual void error(const char *fmt, ...) = 0;
};

/**
 * @brief   A logger that outputs to stdout for info messages and stderr for warnings and errors.
 */
class DefaultLogger: public Logger
{
public:
    /**
     * @brief       Log info messages to stdout.
     * @param fmt   Format string (if additional arguments are passed) or message to display
     */
    virtual void info(const char *fmt, ...)
    {
        std::va_list argptr;
        va_start(argptr, fmt);
        std::vprintf(fmt, argptr);
        va_end(argptr);
    }

    /**
     * @brief       Log warnings to stderr.
     * @param fmt   Format string (if additional arguments are passed) or message to display
     */
    virtual void warning(const char *fmt, ...)
    {
        std::va_list argptr;
        va_start(argptr, fmt);
        std::vfprintf(stderr, fmt, argptr);
        va_end(argptr);
    }

    /**
     * @brief       Log errors to stderr.
     * @param fmt   Format string (if additional arguments are passed) or message to display
     */
    virtual void error(const char *fmt, ...)
    {
        std::va_list argptr;
        va_start(argptr, fmt);
        std::vfprintf(stderr, fmt, argptr);
        va_end(argptr);
    }
};

#endif // CONFIG_H
