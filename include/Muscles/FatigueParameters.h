#ifndef BIORBD_MUSCLES_FATIGUE_PARAMETERS_H
#define BIORBD_MUSCLES_FATIGUE_PARAMETERS_H

#include <memory>
#include "biorbdConfig.h"

#include "Utils/Scalar.h"

namespace biorbd
{
namespace muscles
{

///
/// \brief Class FatigueParameters that holds the muscle fatigue parameters
///
class BIORBD_API FatigueParameters
{
public:

    ///
    /// \brief Construct the fatigue parameters
    /// \param fatigueRate The fatigue rate
    /// \param recoveryRate The recovery rate
    /// \param developFactor The develop factor
    /// \param recoveryFactor The recovery factor
    ///
    FatigueParameters(
        const biorbd::utils::Scalar& fatigueRate = 0,
        const biorbd::utils::Scalar& recoveryRate = 0,
        const biorbd::utils::Scalar& developFactor = 0,
        const biorbd::utils::Scalar& recoveryFactor = 0);

    ///
    /// \brief Deep copy of the fatigue parameters
    /// \return Copy of the fatigue parameters
    ///
    biorbd::muscles::FatigueParameters DeepCopy() const;

    ///
    /// \brief Deep copy of fatigue parameters into another FatigueParameters
    /// \param other The fatigue parameters to copy
    ///
    void DeepCopy(
        const biorbd::muscles::FatigueParameters& other);

    ///
    /// \brief Set the fatigue rate
    /// \param fatigueRate The fatigue rate
    ///
    void setFatigueRate(
        const biorbd::utils::Scalar& fatigueRate);

    // Get and Set
    ///
    /// \brief Return the fatigue rate
    /// \return The fatigue rate
    ///
    const biorbd::utils::Scalar& fatigueRate() const;

    ///
    /// \brief Set the recovery rate
    /// \param recoveryRate The recovery rate
    ///
    void setRecoveryRate(
        const biorbd::utils::Scalar& recoveryRate);

    ///
    /// \brief Return the recovery rate
    /// \return The recovery rate
    ///
    const biorbd::utils::Scalar& recoveryRate() const;

    ///
    /// \brief Set the develop factor
    /// \param developFactor The develop factor
    ///
    void setDevelopFactor(
        const biorbd::utils::Scalar& developFactor);

    ///
    /// \brief Return the develop factor
    /// \return The develop factor
    ///
    const biorbd::utils::Scalar& developFactor() const;

    ///
    /// \brief Set the recovery factor
    /// \param recoveryFactor The recovery factor
    ///
    void setRecoveryFactor(
        const biorbd::utils::Scalar& recoveryFactor);

    ///
    /// \brief Return the recovery factor
    /// \return The recovery factor
    ///
    const biorbd::utils::Scalar& recoveryFactor() const;

protected:
    std::shared_ptr<biorbd::utils::Scalar> m_fatigueRate; ///< The fatigue rate
    std::shared_ptr<biorbd::utils::Scalar> m_recoveryRate;///< The recovery rate
    std::shared_ptr<biorbd::utils::Scalar> m_developFactor; ///<The develop factor
    std::shared_ptr<biorbd::utils::Scalar> m_recoveryFactor; ///<The recovery factor

};

}
}

#endif // BIORBD_MUSCLES_FATIGUE_PARAMETERS_H
