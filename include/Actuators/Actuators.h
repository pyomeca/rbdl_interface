#ifndef BIORBD_ACTUATORS_ACTUATORS_H
#define BIORBD_ACTUATORS_ACTUATORS_H

#include <vector>
#include <memory>
#include "biorbdConfig.h"
#include "Utils/Scalar.h"

namespace biorbd
{
namespace utils
{
class Vector;
}

namespace rigidbody
{
class GeneralizedCoordinates;
class GeneralizedVelocity;
class GeneralizedTorque;
}

namespace actuator
{
class Actuator;
///
/// \brief Class holder for a set of actuators
///
class BIORBD_API Actuators
{
public:
    ///
    /// \brief Construct actuators
    ///
    Actuators();

    ///
    /// \brief Construct actuators from another set of actuators
    /// \param other The other actuators
    ///
    Actuators(
        const biorbd::actuator::Actuators& other);

    ///
    /// \brief Destroy actuators class properly
    ///
    virtual ~Actuators();

    ///
    /// \brief Deep copy of the actuator holder from other actuator holder
    /// \param other The other actuators
    ///
    void DeepCopy(
        const biorbd::actuator::Actuators& other);

    ///
    /// \brief Add an actuator to the set of actuators
    /// \param a The actuator to add
    ///
    void addActuator(
        const biorbd::actuator::Actuator &a);

    ///
    /// \brief Indicate to biorbd to are done adding actuators, sanity checks are performed
    ///
    void closeActuator();

    ///
    /// \brief Return two vectors of max torque (it is impossible to know if eccentric or concentric is required, therefore both are returned)
    /// \param Q The generalized coordinates of the actuators
    /// \param Qdot The generalized velocities of the actuators
    /// \return Two vectors of maximal torque
    ///
    std::pair<biorbd::rigidbody::GeneralizedTorque, biorbd::rigidbody::GeneralizedTorque>
    torqueMax(
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedVelocity& Qdot);

    ///
    /// \brief Return the maximal generalized torque
    /// \param activation The level of activation of the torque. A positive value is interpreted as concentric contraction and negative as eccentric contraction
    /// \param Q The generalized coordinates of the actuators
    /// \param Qdot The generalized velocities of the actuators
    /// \return The maximal generalized torque
    ///
    biorbd::rigidbody::GeneralizedTorque torqueMax(
        const biorbd::utils::Vector &activation,
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedVelocity &Qdot);

    ///
    /// \brief Return the generalized torque
    /// \param activation The level of activation of the torque. A positive value is interpreted as concentric contraction and negative as eccentric contraction
    /// \param Q The generalized coordinates of the actuators
    /// \param Qdot The generalized velocities of the actuators
    /// \return The maximal generalized torque
    ///
    biorbd::rigidbody::GeneralizedTorque torque(
        const biorbd::utils::Vector &activation,
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedVelocity &Qdot);

    // Get and set
    ///
    /// \brief Return a specific concentric/eccentric actuator
    /// \param dof Index of the DoF associated with actuator
    /// \return The actuator
    ///
    const std::pair<std::shared_ptr<biorbd::actuator::Actuator>, std::shared_ptr<biorbd::actuator::Actuator>>&
            actuator(unsigned int dof);

    ///
    /// \brief Return a specific actuator
    /// \param dof Index of the DoF associated with actuator
    /// \param concentric If the return value is the concentric (true) or eccentric (false) value
    /// \return The actuator
    ///
    const biorbd::actuator::Actuator& actuator(unsigned int dof, bool concentric);

    ///
    /// \brief Return the toal number of actuators
    /// \return The total number of actuators
    ///
    unsigned int nbActuators() const;

protected:
    std::shared_ptr<std::vector<std::pair<std::shared_ptr<biorbd::actuator::Actuator>, std::shared_ptr<biorbd::actuator::Actuator>>>>
    m_all; ///<All the actuators reunited /pair (+ or -)
    std::shared_ptr<std::vector<bool>> m_isDofSet;///< If DoF all dof are set
    std::shared_ptr<bool> m_isClose; ///< If the set is ready

    ///
    /// \brief getTorqueMaxDirection Get the max torque of a specific actuator (interface necessary because of CasADi)
    /// \param actuator The actuator to gather from
    /// \param Q The Generalized coordinates
    /// \param Qdot The Generalized velocity
    /// \return The torque max
    ///
    biorbd::utils::Scalar getTorqueMaxDirection(
        const std::shared_ptr<Actuator> actuator,
        const rigidbody::GeneralizedCoordinates &Q,
        const rigidbody::GeneralizedVelocity &Qdot) const;

};

}
}

#endif // BIORBD_ACTUATORS_ACTUATORS_H
