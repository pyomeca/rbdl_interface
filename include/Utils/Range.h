#ifndef BIORBD_UTILS_RANGE_H
#define BIORBD_UTILS_RANGE_H

#include "biorbdConfig.h"

#include <memory>

namespace biorbd
{
namespace utils
{

///
/// \brief Class Range
///
class BIORBD_API Range
{
public:

    ///
    /// \brief Construct generalized coordinates range
    ///
    Range(
        double min = -M_PI,
        double max = M_PI);

    ///
    /// \brief Deep copy of the Range
    /// \return Deep copy of the Range
    ///
    biorbd::utils::Range DeepCopy() const;

    ///
    /// \brief Deep copy of Range
    /// \param other The Range to copy
    ///
    void DeepCopy(const biorbd::utils::Range& other);

    ///
    /// \brief Set a new value for the minimum
    /// \param min The value to set to
    ///
    void setMin(double min);

    ///
    /// \brief Return the minimum value
    /// \return The minimum value
    ///
    double min() const;

    ///
    /// \brief Set a new value for the maximal
    /// \param max The value to set to
    ///
    void setMax(double max);

    ///
    /// \brief Return the maximum value
    /// \return The minimum value
    ///
    double max() const;

protected:
    std::shared_ptr<double> m_min; ///< The minimal value allowed by the range
    std::shared_ptr<double> m_max; ///< The maximal value allowed by the range
};

}
}

#endif // BIORBD_UTILS_RANGE_H
