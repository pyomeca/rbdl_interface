#ifndef BIORBD_UTILS_TIMER_H
#define BIORBD_UTILS_TIMER_H

#include <memory>
#include <ctime>
#include "biorbdConfig.h"

namespace biorbd
{
namespace utils
{

///
/// \brief Wrapper around timer function in C++
///
class BIORBD_API Timer
{
public:
    ///
    /// \brief Construct timer
    /// \param startNow If the timer should start now
    ///
    Timer(
        bool startNow = false);

    ///
    /// \brief Start the timer
    ///
    void start();

    ///
    /// \brief Return if the timer is started
    /// \return If the timer is started
    ///
    bool isStarted();

    ///
    /// \brief Return if the timer is paused
    /// \return If the timer is paused
    ///
    void pause();

    ///
    /// \brief Resume the timer. If it is paused, it adds a lap
    ///
    void resume();

    ///
    /// \brief Get the lap time of a running timer. Return 0 if the timer is not running
    /// \return The lap time
    ///
    double getLap();

    ///
    /// \brief Stop the timer and return the lap time
    ///
    double stop();

protected:
    ///
    /// \brief Add a pause to the timer time (total time being execution time - pause time)
    ///
    void addPauseTime();

    ///
    /// \brief Get the time spent until start
    /// \param timer The timer to get the time from
    /// \return The time
    ///
    double getTime(const std::clock_t& timer);

    bool m_isStarted; ///< If the timer is started
    bool m_isPaused; ///< If the timer is paused
    std::clock_t m_start; ///< The start time
    std::clock_t m_pauseTime; ///< The pause time
    double m_totalPauseTime; ///< The total pause time

};

}
}

#endif // BIORBD_UTILS_TIMER_H
