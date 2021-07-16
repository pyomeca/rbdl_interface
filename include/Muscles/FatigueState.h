#ifndef BIORBD_MUSCLES_FATIGUE_STATE_H
#define BIORBD_MUSCLES_FATIGUE_STATE_H

#include <memory>
#include "biorbdConfig.h"
#include "MusclesEnums.h"
#include "Utils/Scalar.h"

namespace biorbd
{
namespace muscles
{

///
/// \brief Base class for Fatigue states models
///
class BIORBD_API FatigueState
{
public:
    ///
    /// \brief Construct fatigue state
    /// \param active Active muscle
    /// \param fatigued Muscle fatigue
    /// \param resting Resting muscle
    ///
    FatigueState(
        const biorbd::utils::Scalar& active = 1,
        const biorbd::utils::Scalar& fatigued = 0,
        const biorbd::utils::Scalar& resting = 0);

    ///
    /// \brief Construct fatigue state from another fatigue state
    /// \param other The other fatigure state
    ///
    FatigueState(
        const biorbd::muscles::FatigueState& other);

    ///
    /// \brief Construct fatigue state from another fatigue state
    /// \param other The other fatigure state
    ///
    FatigueState(
        const std::shared_ptr<biorbd::muscles::FatigueState> other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~FatigueState();

    ///
    /// \brief Deep copy of the fatigue state
    /// \return A deep copy of the fatigue state
    ///
    biorbd::muscles::FatigueState DeepCopy() const;

    ///
    /// \brief Deep copy of fatigue state in another fatigue state
    /// \param other The fatigue state to copy
    ///
    void DeepCopy(
        const biorbd::muscles::FatigueState& other);

#ifndef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Safe set of the state (ensure that sum of all activation is 1)
    /// \param active Proportion of the activated fibers in the muscle
    /// \param fatigued Proportion of the fatigued fibers in the muscle
    /// \param resting Proportion of the resting fibers in the muscle
    /// \param turnOffWarnings If the warnings should be OFF or ON.
    ///
    virtual void setState(
        biorbd::utils::Scalar active,
        biorbd::utils::Scalar fatigued,
        biorbd::utils::Scalar resting,
        bool turnOffWarnings = false);
#endif

    ///
    /// \brief Return the propotion of active muscle fibers
    /// \return The propotion of active muscle fibers
    ///
    const biorbd::utils::Scalar& activeFibers() const;

    ///
    /// \brief Return the propotion of fatigued muscle fibers
    /// \return The propotion of fatigued muscle fibers
    ///
    const biorbd::utils::Scalar& fatiguedFibers() const;

    ///
    /// \brief Return the propotion of resting muscle fibers
    /// \return The propotion of resting muscle fibers
    ///
    const biorbd::utils::Scalar& restingFibers() const;

    ///
    /// \brief Return the type of muscle fatigue model
    /// \return The type of muscle fatigue model
    ///
    biorbd::muscles::STATE_FATIGUE_TYPE getType() const;

protected:
    std::shared_ptr<biorbd::utils::Scalar>
    m_activeFibers; ///< Proportion of active muscle fibers
    std::shared_ptr<biorbd::utils::Scalar>
    m_fatiguedFibers;///< Proportion of fatigued muscle fibers
    std::shared_ptr<biorbd::utils::Scalar>
    m_restingFibers;///< Proportion of resting muscle fibers

    ///
    /// \brief Set the type
    ///
    virtual void setType();
    std::shared_ptr<biorbd::muscles::STATE_FATIGUE_TYPE>
    m_type; ///< Type of the muscle fatigue model

};

}
}

#endif // BIORBD_MUSCLES_FATIGUE_STATE_H
