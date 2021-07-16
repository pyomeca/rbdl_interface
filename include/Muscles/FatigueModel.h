#ifndef BIORBD_MUSCLES_FATIGUE_MODEL_H
#define BIORBD_MUSCLES_FATIGUE_MODEL_H

#include <memory>
#include "biorbdConfig.h"
#include "MusclesEnums.h"
#include "Utils/Scalar.h"

namespace biorbd
{
namespace utils
{
class String;
}

namespace muscles
{
class Muscle;
class StateDynamics;
class FatigueState;

///
/// \brief Class Fatigue model of a muscle
///
class BIORBD_API FatigueModel
{
public:
    ///
    /// \brief Contruct a fatigable muscle model
    /// \param dynamicFatigueType The type of fatigue of the muscle
    ///
    FatigueModel(
        biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType);

    ///
    /// \brief Construct a fatigable muscle model from another muscle
    /// \param other The other fatigue model
    ///
    FatigueModel(
        const biorbd::muscles::FatigueModel& other);

    ///
    /// \brief Construct a fatigable muscle model from another muscle
    /// \param other The other fatigue model
    ///
    FatigueModel(
        const std::shared_ptr<biorbd::muscles::FatigueModel> other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~FatigueModel() = 0;

    ///
    /// \brief Deep copy of the fatigue model
    /// \param other The other fatigue model to copy
    ///
    void DeepCopy(
        const biorbd::muscles::FatigueModel& other);

    ///
    /// \brief Compute the time derivative state
    /// \param emg EMG data
    ///
    virtual void computeTimeDerivativeState(
        const biorbd::muscles::StateDynamics& emg);

#ifndef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Set the fatigue state
    /// \param active
    /// \param fatigued
    /// \param resting
    virtual void setFatigueState(
        const biorbd::utils::Scalar& active,
        const biorbd::utils::Scalar& fatigued,
        const biorbd::utils::Scalar& resting);
#endif

    ///
    /// \brief Return the fatigue state
    /// \return The fatigue state
    ///
    biorbd::muscles::FatigueState& fatigueState();
    ///
    /// \brief Return the fatigue state
    /// \return The fatigue state
    ///
    const biorbd::muscles::FatigueState& fatigueState() const;


protected:
    std::shared_ptr<biorbd::muscles::FatigueState>
    m_fatigueState; ///< The fatigue state

};

}
}

#endif // BIORBD_MUSCLES_FATIGUE_MODEL_H
