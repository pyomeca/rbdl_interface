#ifndef BIORBD_ACTUATORS_ACTUATOR_CONSTANT_H
#define BIORBD_ACTUATORS_ACTUATOR_CONSTANT_H

#include "biorbdConfig.h"
#include "Actuators/Actuator.h"

namespace biorbd
{
namespace actuator
{

///
/// \brief Class ActuatorConstant is a joint actuator type which maximum is contant
///
class BIORBD_API ActuatorConstant : public Actuator
{
public:
    ///
    /// \brief Construct a constant actuator
    ///
    ActuatorConstant();

    ///
    /// \brief Construct a constant actuator from another actuator
    /// \param other The other constant actuator
    ///
    ActuatorConstant(
        const biorbd::actuator::ActuatorConstant& other);

    ///
    /// \brief Construct a constant actuator
    /// \param direction The direction of the actuator (+1 or -1)
    /// \param Tmax The maximum torque that can be done
    /// \param dofIdx Index of the DoF associated with actuator
    ///
    ActuatorConstant(
        int direction,
        const biorbd::utils::Scalar& Tmax,
        unsigned int dofIdx);

    ///
    /// \brief Construct a constant actuator
    /// \param direction The direction of the actuator (+1 or -1)
    /// \param Tmax The maximum torque that can be done
    /// \param dofIdx Index of the DoF associated with actuator
    /// \param jointName The name of the parent joint
    ///
    ActuatorConstant(
        int direction,
        const biorbd::utils::Scalar& Tmax,
        unsigned int dofIdx,
        const biorbd::utils::String &jointName);

    ///
    /// \brief Deep copy of the constant actuator
    /// \return A copy of the constant actuator
    ///
    biorbd::actuator::ActuatorConstant DeepCopy() const;

    ///
    /// \brief Deep copy of the constant actuator to another constant actuator
    /// \param other The constant actuator to copy
    ///
    void DeepCopy(
        const biorbd::actuator::ActuatorConstant& other);

    ///
    /// \brief Return the maximal torque
    /// \return The maximal torque
    ///
    virtual biorbd::utils::Scalar torqueMax();

protected:
    ///
    /// \brief Set the type of the constant actuator
    ///
    virtual void setType();
    std::shared_ptr<biorbd::utils::Scalar>
    m_Tmax; ///< Maximal torque that can be done

};

}
}

#endif // BIORBD_ACTUATORS_ACTUATOR_CONSTANT_H
