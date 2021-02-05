#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <sys/time.h>
#include <cstdarg>
#include <cmath>

// for loging
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
    /**
     * @brief       Pure virtual function to log errors.
     * @param fmt   Format string (if additional arguments are passed) or message to display
     */
    // virtual void thread(const char *fmt, const char *threadName, ...) = 0;

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

#endif