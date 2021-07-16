#ifndef BIORBD_UTILS_BENCHMARK_H
#define BIORBD_UTILS_BENCHMARK_H

#include <memory>
#include <map>
#include "biorbdConfig.h"

namespace biorbd
{
namespace utils
{
class String;
class Timer;

///
/// \brief Collection of method to supercifically benchmark the code
///
/// Benchmark can run multiple timers at the same time. One must simply gives
/// different names
///
class BIORBD_API Benchmark
{
public:
    ///
    /// \brief Construct a benchmark analysis
    ///
    Benchmark();

    ///
    /// \brief Start the timer of a specified name
    /// \param name The name to associate with timer
    /// \param force If force is true, the timer starts even though it is already started
    ///
    void startTimer(
        const biorbd::utils::String& name,
        bool force = false);

    ///
    /// \brief Pause a timer of a specified name
    /// \param name Name of the timer to pause
    ///
    void pauseTimer(
        const biorbd::utils::String& name);

    ///
    /// \brief Resume a timer of a specified name
    /// \param name Name of the timer to resume
    ///
    void resumeTimer(
        const biorbd::utils::String& name);

    ///
    /// \brief Get lap time of a specified timer
    /// \param name Name of the timer
    /// \return The lap time of a specified timer
    ///
    double getLap(
        const biorbd::utils::String& name);

    ///
    /// \brief Stop the timer of a specified name and get lap time
    /// \param name Name of the timer
    /// \return The lap time of a specified timer
    ///
    double stopTimer(
        const biorbd::utils::String& name);

    ///
    /// \brief To waste time (similar to a sleep function)
    /// \param timeInSec Time in seconds to wait
    ///
    static void wasteTime(double timeInSec);

    ///
    /// \brief Add a new timer to the timer set
    /// \param name The name of the timer to add
    ///
    void addTimer(
        const biorbd::utils::String& name);

    ///
    /// \brief Get the index of the specified timer
    /// \param name The name of the timer
    /// \return The index of the specified timer
    ///
    int getTimerIdx(
        const biorbd::utils::String& name);

protected:
    std::map<biorbd::utils::String, biorbd::utils::Timer> m_timers;///< Timers
    std::map<biorbd::utils::String, int> m_counts;///< Counts

};

}
}

#endif // BIORBD_UTILS_BENCHMARK_H
