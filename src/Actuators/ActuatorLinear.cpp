#define BIORBD_API_EXPORTS
#include "Actuators/ActuatorLinear.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"

biorbd::actuator::ActuatorLinear::ActuatorLinear() :
    Actuator(),
    m_m(std::make_shared<biorbd::utils::Scalar>(0)),
    m_b(std::make_shared<biorbd::utils::Scalar>(0))
{
    setType();
}

biorbd::actuator::ActuatorLinear::ActuatorLinear(
    const biorbd::actuator::ActuatorLinear &other) :
    Actuator(other),
    m_m(other.m_m),
    m_b(other.m_b)
{

}

biorbd::actuator::ActuatorLinear::ActuatorLinear(
    int direction,
    const biorbd::utils::Scalar& T0,
    const biorbd::utils::Scalar& slope,
    unsigned int dofIdx) :
    Actuator(direction, dofIdx),
    m_m(std::make_shared<biorbd::utils::Scalar>(slope)),
    m_b(std::make_shared<biorbd::utils::Scalar>(T0))
{
    setType();
}

biorbd::actuator::ActuatorLinear::ActuatorLinear(
    int direction,
    const biorbd::utils::Scalar& T0,
    const biorbd::utils::Scalar& slope,
    unsigned int dofIdx,
    const biorbd::utils::String &jointName) :
    Actuator(direction, dofIdx, jointName),
    m_m(std::make_shared<biorbd::utils::Scalar>(slope)),
    m_b(std::make_shared<biorbd::utils::Scalar>(T0))
{
    setType();
}

biorbd::actuator::ActuatorLinear::~ActuatorLinear()
{

}

biorbd::actuator::ActuatorLinear biorbd::actuator::ActuatorLinear::DeepCopy()
const
{
    biorbd::actuator::ActuatorLinear copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::actuator::ActuatorLinear::DeepCopy(const
        biorbd::actuator::ActuatorLinear &other)
{
    biorbd::actuator::Actuator::DeepCopy(other);
    *m_m = *other.m_m;
    *m_b = *other.m_b;
}

biorbd::utils::Scalar biorbd::actuator::ActuatorLinear::torqueMax()
{
    biorbd::utils::Error::raise("torqueMax for ActuatorLinear must be called with Q and Qdot");
}


biorbd::utils::Scalar biorbd::actuator::ActuatorLinear::torqueMax(
    const biorbd::rigidbody::GeneralizedCoordinates &Q) const
{
    return (Q[*m_dofIdx]*180/M_PI) * *m_m + *m_b;
}

void biorbd::actuator::ActuatorLinear::setType()
{
    *m_type = biorbd::actuator::TYPE::LINEAR;
}
