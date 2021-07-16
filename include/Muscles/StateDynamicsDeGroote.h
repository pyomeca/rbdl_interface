#ifndef BIORBD_MUSCLES_STATE_DYNAMICS_DE_GROOTE_H
#define BIORBD_MUSCLES_STATE_DYNAMICS_DE_GROOTE_H

#include "biorbdConfig.h"
#include "Muscles/StateDynamics.h"

namespace biorbd
{
namespace muscles
{
///
/// \brief EMG with the capability to compute the time derivative
///
class BIORBD_API StateDynamicsDeGroote : public biorbd::muscles::StateDynamics
{
public:
    ///
    /// \brief Construct the state dynamics
    /// \param excitation The muscle excitation
    /// \param activation The muscle activation
    ///
    StateDynamicsDeGroote(
        const biorbd::utils::Scalar& excitation = 0,
        const biorbd::utils::Scalar& activation = 0);

    ///
    /// \brief Construct a state dynamics from another state dynamics
    /// \param other The other state dynamics
    ///
    StateDynamicsDeGroote(
        const biorbd::muscles::StateDynamicsDeGroote& other);

    ///
    /// \brief Deep copy of state dynamics
    /// \return A deep copy of state dynamics
    ///
    biorbd::muscles::StateDynamicsDeGroote DeepCopy() const;

    ///
    /// \brief Deep copy of state dynamics into another state dynamics
    /// \param other The state dynamics to copy
    ///
    void DeepCopy(
        const biorbd::muscles::StateDynamicsDeGroote& other);

    ///
    /// \brief Compute and return the activation time derivative
    /// \param characteristics The muscle characteristics
    /// \param alreadyNormalized If already normalized
    /// \return The activation time derivative
    ///
    virtual const biorbd::utils::Scalar& timeDerivativeActivation(
        const Characteristics& characteristics,
        bool alreadyNormalized = false);

protected:
    virtual void setType();

};

}
}

#endif // BIORBD_MUSCLES_STATE_DYNAMICS_DE_GROOTE_H
