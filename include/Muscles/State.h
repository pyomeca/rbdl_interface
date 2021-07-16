#ifndef BIORBD_MUSCLES_STATE_H
#define BIORBD_MUSCLES_STATE_H

#include <memory>
#include "biorbdConfig.h"
#include "Muscles/MusclesEnums.h"
#include "Utils/Scalar.h"

namespace biorbd
{

namespace muscles
{
///
/// \brief EMG holder to interact with the muscle
///
class BIORBD_API State
{
public:
    ///
    /// \brief Construct a state
    /// \param excitation The muscle excitation
    /// \param activation The muscle activation
    ///
    State(
        const biorbd::utils::Scalar& excitation = 0,
        const biorbd::utils::Scalar& activation = 0);

    ///
    /// \brief Construct a muscle state from another state
    /// \param other The other state
    ///
    State(
        const biorbd::muscles::State& other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~State();

    ///
    /// \brief Deep copy of state
    /// \return A deep copy of state
    ///
    biorbd::muscles::State DeepCopy() const;

    ///
    /// \brief Deep copy of state into another state
    /// \param other The state to copy
    ///
    void DeepCopy(
        const biorbd::muscles::State& other);

    ///
    /// \brief Set the muscle excitation
    /// \param val The value of the muscle excitation
    /// \param turnOffWarnings If the warnings should be OFF or ON.
    ///
    /// Even if the warning on the excitation being lower than 0 is set to OFF
    /// it changes it to 0 anyway, but doesn't send a warning saying it.
    ///
    virtual void setExcitation(
        const biorbd::utils::Scalar& val,
        bool turnOffWarnings = false);

    ///
    /// \brief Return the muscle excitation
    /// \return The muscle excitation
    ///
    const biorbd::utils::Scalar& excitation() const;

    ///
    /// \brief Compute and return the normalized excitation
    /// \param emgMax The maximal emg
    /// \param turnOffWarnings If the warning when excitation is higher than 1 should be off or on
    /// \return The normalized excitation
    ///
    /// Even when the warning is ON, the computation is performed anyway
    ///
    const biorbd::utils::Scalar& normalizeExcitation(
        const biorbd::muscles::State &emgMax,
        bool turnOffWarnings = false);

    ///
    /// \brief Force set the normalized excitation
    /// \param val Value of the normalized excitation to set
    ///
    void setExcitationNorm(
        const biorbd::utils::Scalar& val);

    ///
    /// \brief Return the previously normalized excitation
    /// \return The normalized excitation
    ///
    const biorbd::utils::Scalar& excitationNorm() const;

    ///
    /// \brief Set the muscle activation
    /// \param val The value of the muscle activation
    /// \param turnOffWarnings If the warnings should be OFF or ON.
    ///
    /// Even if the warning on the activation being lower than 0 is set to OFF
    /// it changes it to 0 anyway, but doesn't send a warning saying it.
    ///
    /// Even if the warning on the activation being higher than 1 is set to OFF
    /// it changes it to 1 anyway, but doesn't send a warning saying it.
    ///
    virtual void setActivation(
        const biorbd::utils::Scalar& val,
        bool turnOffWarnings = false);

    ///
    /// \brief Return the muscle activation
    /// \return The muscle activation
    ///
    const biorbd::utils::Scalar& activation() const;

    ///
    /// \brief Return the state type
    /// \return The state type
    ///
    biorbd::muscles::STATE_TYPE type() const;
protected:
    ///
    /// \brief Set the type to simple_state
    ///
    virtual void setType();

    std::shared_ptr<biorbd::muscles::STATE_TYPE> m_stateType;///< The state type
    std::shared_ptr<biorbd::utils::Scalar> m_excitation;///< The muscle excitation
    std::shared_ptr<biorbd::utils::Scalar>
    m_excitationNorm; ///< The normalized excitation
    std::shared_ptr<biorbd::utils::Scalar> m_activation;///< The muscle activation

};

}
}

#endif // BIORBD_MUSCLES_STATE_H
