#ifndef BIORBD_MUSCLES_STATE_DYNAMICS_H
#define BIORBD_MUSCLES_STATE_DYNAMICS_H

#include "biorbdConfig.h"
#include "Muscles/State.h"

namespace biorbd
{
namespace muscles
{
class Characteristics;
///
/// \brief EMG with the capability to compute the time derivative
///
class BIORBD_API StateDynamics : public biorbd::muscles::State
{
public:
    ///
    /// \brief Construct the state dynamics
    /// \param excitation The muscle excitation
    /// \param activation The muscle activation
    ///
    StateDynamics(
        const biorbd::utils::Scalar& excitation = 0,
        const biorbd::utils::Scalar& activation = 0);

    ///
    /// \brief Construct a state dynamics from another state dynamics
    /// \param other The other state dynamics
    ///
    StateDynamics(
        const biorbd::muscles::State& other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~StateDynamics();

    ///
    /// \brief Deep copy of state dynamics
    /// \return A deep copy of state dynamics
    ///
    biorbd::muscles::StateDynamics DeepCopy() const;

    ///
    /// \brief Deep copy of state dynamics into another state dynamics
    /// \param other The state dynamics to copy
    ///
    void DeepCopy(
        const biorbd::muscles::StateDynamics& other);

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
    /// \brief Return the previous activation
    /// \return The previous activation
    ///
    const biorbd::utils::Scalar& previousExcitation() const;

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
    /// \brief Return the previous activation
    /// \return The previous activation
    ///
    const biorbd::utils::Scalar& previousActivation() const;

    ///
    /// \brief Compute and return the activation time derivative from the excitation and activation
    /// \param excitation The muscle excitation
    /// \param activation The muscle activation
    /// \param characteristics The muscle characteristics
    /// \param alreadyNormalized If already normalized
    /// \return The activation time derivative
    ///
    virtual const biorbd::utils::Scalar& timeDerivativeActivation(
        const biorbd::utils::Scalar& excitation,
        const biorbd::utils::Scalar& activation,
        const Characteristics& characteristics,
        bool alreadyNormalized = false);

    ///
    /// \brief Compute and return the activation time derivative
    /// \param emg The emg
    /// \param characteristics The muscle characteristics
    /// \param alreadyNormalized If already normalized
    /// \return The activation time derivative
    ///
    virtual const biorbd::utils::Scalar& timeDerivativeActivation(
        const biorbd::muscles::State& emg,
        const biorbd::muscles::Characteristics& characteristics,
        bool alreadyNormalized = false);

    ///
    /// \brief Compute and return the activation time derivative
    /// \param characteristics The muscle characteristics
    /// \param alreadyNormalized If already normalized
    /// \return The activation time derivative
    ///
    virtual const biorbd::utils::Scalar& timeDerivativeActivation(
        const biorbd::muscles::Characteristics& characteristics,
        bool alreadyNormalized = false);

    ///
    /// \brief Return the previously computed activation time derivative
    /// \return The activation time derivative
    ///
    virtual const biorbd::utils::Scalar& timeDerivativeActivation();

protected:
    virtual void setType();
    std::shared_ptr<biorbd::utils::Scalar>
    m_previousExcitation; ///< The previous excitation
    std::shared_ptr<biorbd::utils::Scalar>
    m_previousActivation; ///<The previous activation
    std::shared_ptr<biorbd::utils::Scalar>
    m_activationDot;///< The activation velocity

};

}
}

#endif // BIORBD_MUSCLES_STATE_DYNAMICS_H
