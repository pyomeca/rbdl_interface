#define BIORBD_API_EXPORTS
#include "Utils/Benchmark.h"

#include "Utils/Timer.h"
#include "Utils/String.h"

biorbd::utils::Benchmark::Benchmark() :
    m_timers(std::map<biorbd::utils::String, biorbd::utils::Timer>()),
    m_counts(std::map<biorbd::utils::String, int>())
{

}


void biorbd::utils::Benchmark::startTimer(
    const biorbd::utils::String& name,
    bool force)
{
    if (force) {
        m_timers[name].start();
    } else if (!m_timers[name].isStarted()) {
        m_timers[name].start();
    }
}

double biorbd::utils::Benchmark::getLap(
    const biorbd::utils::String& name)
{
    return m_timers[name].getLap();
}

double biorbd::utils::Benchmark::stopTimer(
    const biorbd::utils::String& name)
{
    return m_timers[name].stop();
}

void biorbd::utils::Benchmark::pauseTimer(
    const biorbd::utils::String& name)
{
    m_timers[name].pause();
}

void biorbd::utils::Benchmark::resumeTimer(
    const biorbd::utils::String& name)
{
    m_timers[name].resume();
}

void biorbd::utils::Benchmark::addTimer(
    const biorbd::utils::String& name)
{
    ++(m_counts[name]);
}

int biorbd::utils::Benchmark::getTimerIdx(
    const biorbd::utils::String& name)
{
    return m_counts[name];
}

void biorbd::utils::Benchmark::wasteTime(
    double seconds)
{
    // Wait for seconds ask doing dummy stuff

    std::clock_t start(std::clock());

    while (((static_cast<double>(std::clock() - start)) / CLOCKS_PER_SEC)<seconds) {
    }

}
