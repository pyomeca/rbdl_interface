#define BIORBD_API_EXPORTS
#include "Actuators/ActuatorConstant.h"

#include "Utils/String.h"
#include "RigidBody/GeneralizedCoordinates.h"

biorbd::actuator::ActuatorConstant::ActuatorConstant() :
    Actuator(),
    m_Tmax(std::make_shared<biorbd::utils::Scalar>(0))
{
    setType();
}

biorbd::actuator::ActuatorConstant::ActuatorConstant(
    const biorbd::actuator::ActuatorConstant &other) :
    Actuator(other),
    m_Tmax(other.m_Tmax)
{

}

biorbd::actuator::ActuatorConstant::ActuatorConstant(
    int direction,
    const biorbd::utils::Scalar& Tmax,
    unsigned int dofIdx) :
    Actuator(direction, dofIdx),
    m_Tmax(std::make_shared<biorbd::utils::Scalar>(Tmax))
{
    setType();
}

biorbd::actuator::ActuatorConstant::ActuatorConstant(
    int direction,
    const biorbd::utils::Scalar& Tmax,
    unsigned int dofIdx,
    const biorbd::utils::String &jointName) :
    Actuator(direction, dofIdx, jointName),
    m_Tmax(std::make_shared<biorbd::utils::Scalar>(Tmax))
{
    setType();
}

biorbd::actuator::ActuatorConstant
biorbd::actuator::ActuatorConstant::DeepCopy() const
{
    biorbd::actuator::ActuatorConstant copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::actuator::ActuatorConstant::DeepCopy(const
        biorbd::actuator::ActuatorConstant &other)
{
    biorbd::actuator::Actuator::DeepCopy(other);
    *m_Tmax = *other.m_Tmax;
}

biorbd::utils::Scalar biorbd::actuator::ActuatorConstant::torqueMax()
{
    return *m_Tmax;
}

void biorbd::actuator::ActuatorConstant::setType()
{
    *m_type = biorbd::actuator::TYPE::CONSTANT;
}
