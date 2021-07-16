#ifndef BIORBD_ACTUATORS_ACTUATOR_H
#define BIORBD_ACTUATORS_ACTUATOR_H

#include <memory>
#include "biorbdConfig.h"
#include "Actuators/ActuatorEnums.h"
#include "Utils/Scalar.h"

namespace biorbd
{
namespace utils
{
class String;
}

namespace actuator
{

///
/// \brief Class Actuator
///
class BIORBD_API Actuator
{
public:

    ///
    /// \brief Construct actuator
    ///
    Actuator();

    ///
    /// \brief Construct actuator from another actuator
    /// \param other The other actuator
    ///
    Actuator(
        const biorbd::actuator::Actuator& other);

    ///
    /// \brief Construct actuator
    /// \param direction Direction of the actuator (+1 or -1)
    /// \param dofIdx Index of the DoF associated with actuator
    ///
    Actuator(
        int direction,
        unsigned int dofIdx);

    ///
    /// \brief Construct actuator
    /// \param direction Direction of the actuator (+1 or -1)
    /// \param dofIdx Index of the DoF associated with actuator
    /// \param jointName The name of the parent joint
    ///
    Actuator(
        int direction,
        unsigned int dofIdx,
        const biorbd::utils::String &jointName);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~Actuator();

    ///
    /// \brief Deep copy of actuator
    /// \param other The actuator to copy
    ///
    void DeepCopy(
        const biorbd::actuator::Actuator& other);

    ///
    /// \brief Return the index of the DoF associated with actuator
    /// \return The index of the DoF associated with actuator
    ///
    unsigned int index() const;

    ///
    /// \brief Return the direction of the actuator
    /// \return The direction of the actuator (+1 or -1)
    ///
    int direction() const;

    ///
    /// \brief Return the type of the actuator
    /// \return The type of the actuator
    ///
    biorbd::actuator::TYPE type() const;

    ///
    /// \brief Return the maximal torque
    /// \return The maximal torque
    ///
    virtual biorbd::utils::Scalar torqueMax() = 0;

protected:
    ///
    /// \brief Set the type of actuator
    ///
    virtual void setType() = 0;

    std::shared_ptr<biorbd::actuator::TYPE> m_type; ///< The type of the actuator
    std::shared_ptr<int> m_direction; ///< The direction of the actuator (+1 or -1)

    std::shared_ptr<biorbd::utils::String>
    m_jointName; ///< Name of the parent joint
    std::shared_ptr<unsigned int>
    m_dofIdx;///< Index of the DoF associated with the actuator

};

}
}

#endif // BIORBD_ACTUATORS_ACTUATOR_H
