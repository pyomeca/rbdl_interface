#define BIORBD_API_EXPORTS
#include "Muscles/IdealizedActuator.h"

#include "Utils/String.h"
#include "Muscles/Characteristics.h"
#include "Muscles/State.h"

biorbd::muscles::IdealizedActuator::IdealizedActuator() :
    biorbd::muscles::Muscle()
{
    setType();
}

biorbd::muscles::IdealizedActuator::IdealizedActuator(
    const biorbd::utils::String& name,
    const biorbd::muscles::Geometry& geometry,
    const biorbd::muscles::Characteristics& characteristics) :
    biorbd::muscles::Muscle(name,geometry,characteristics)
{
    setType();
}

biorbd::muscles::IdealizedActuator::IdealizedActuator(
    const biorbd::utils::String &name,
    const biorbd::muscles::Geometry &geometry,
    const biorbd::muscles::Characteristics &characteristics,
    const biorbd::muscles::State &emg) :
    biorbd::muscles::Muscle(name,geometry,characteristics,emg)
{
    setType();
}

biorbd::muscles::IdealizedActuator::IdealizedActuator(
    const biorbd::utils::String &name,
    const biorbd::muscles::Geometry &geometry,
    const biorbd::muscles::Characteristics &characteristics,
    const biorbd::muscles::PathModifiers &pathModifiers) :
    biorbd::muscles::Muscle(name,geometry,characteristics, pathModifiers)
{
    setType();
}

biorbd::muscles::IdealizedActuator::IdealizedActuator(
    const biorbd::utils::String& name,
    const biorbd::muscles::Geometry& geometry,
    const biorbd::muscles::Characteristics& characteristics,
    const biorbd::muscles::PathModifiers &pathModifiers,
    const biorbd::muscles::State& emg) :
    biorbd::muscles::Muscle(name,geometry,characteristics,pathModifiers,emg)
{
    setType();
}

biorbd::muscles::IdealizedActuator::IdealizedActuator(const
        biorbd::muscles::Muscle &other) :
    biorbd::muscles::Muscle (other)
{

}

biorbd::muscles::IdealizedActuator::IdealizedActuator(const
        std::shared_ptr<biorbd::muscles::Muscle> other) :
    biorbd::muscles::Muscle (other)
{

}

biorbd::muscles::IdealizedActuator
biorbd::muscles::IdealizedActuator::DeepCopy() const
{
    biorbd::muscles::IdealizedActuator copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::IdealizedActuator::DeepCopy(const
        biorbd::muscles::IdealizedActuator &other)
{
    biorbd::muscles::Muscle::DeepCopy(other);
}

const biorbd::utils::Scalar& biorbd::muscles::IdealizedActuator::force(
    const biorbd::muscles::State &emg)
{
    computeForce(emg);
    return *m_force;
}

const biorbd::utils::Scalar& biorbd::muscles::IdealizedActuator::force(
    biorbd::rigidbody::Joints &,
    const biorbd::rigidbody::GeneralizedCoordinates &,
    const biorbd::rigidbody::GeneralizedVelocity &,
    const biorbd::muscles::State &emg,
    int)
{
    computeForce(emg);
    return *m_force;
}

const biorbd::utils::Scalar& biorbd::muscles::IdealizedActuator::force(
    biorbd::rigidbody::Joints &,
    const biorbd::rigidbody::GeneralizedCoordinates &,
    const biorbd::muscles::State &emg,
    int)
{
    computeForce(emg);
    return *m_force;
}

biorbd::utils::Scalar
biorbd::muscles::IdealizedActuator::getForceFromActivation(
    const biorbd::muscles::State &emg)
{
    return characteristics().forceIsoMax() * (emg.activation());
}

void biorbd::muscles::IdealizedActuator::setType()
{
    *m_type = biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR;
}
