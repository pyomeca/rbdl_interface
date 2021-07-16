#define BIORBD_API_EXPORTS
#include "Utils/Range.h"

biorbd::utils::Range::Range(
    double min,
    double max) :
    m_min(std::make_shared<double>(min)),
    m_max(std::make_shared<double>(max))
{

}

biorbd::utils::Range
biorbd::utils::Range::DeepCopy() const
{
    biorbd::utils::Range copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::utils::Range::DeepCopy(
    const biorbd::utils::Range &other)
{
    *m_min = *other.m_min;
    *m_max = *other.m_max;
}

void biorbd::utils::Range::setMin(
    double min)
{
    *m_min = min;
}

double biorbd::utils::Range::min() const
{
    return *m_min;
}

void biorbd::utils::Range::setMax(
    double max)
{
    *m_max = max;
}

double biorbd::utils::Range::max() const
{
    return *m_max;
}
