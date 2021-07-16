#define BIORBD_API_EXPORTS
#include "Muscles/HillThelenActiveOnlyType.h"

#include <math.h>
#include "Utils/String.h"
#include "Muscles/Geometry.h"
#include "Muscles/Characteristics.h"

biorbd::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType() :
    biorbd::muscles::HillThelenType()
{
    setType();
}
biorbd::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const biorbd::utils::String& name,
    const biorbd::muscles::Geometry& geometry,
    const biorbd::muscles::Characteristics& characteristics) :
    biorbd::muscles::HillThelenType (name, geometry, characteristics)
{
    setType();
}

biorbd::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const biorbd::utils::String &name,
    const biorbd::muscles::Geometry &geometry,
    const biorbd::muscles::Characteristics &characteristics,
    const biorbd::muscles::State &emg) :
    biorbd::muscles::HillThelenType (name, geometry, characteristics, emg)
{
    setType();
}

biorbd::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const biorbd::utils::String &name,
    const biorbd::muscles::Geometry &geometry,
    const biorbd::muscles::Characteristics &characteristics,
    const biorbd::muscles::PathModifiers &pathModifiers) :
    biorbd::muscles::HillThelenType (name, geometry, characteristics, pathModifiers)
{
    setType();
}

biorbd::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const biorbd::utils::String& name,
    const biorbd::muscles::Geometry& geometry,
    const biorbd::muscles::Characteristics& characteristics,
    const biorbd::muscles::PathModifiers &pathModifiers,
    const biorbd::muscles::State& emg) :
    biorbd::muscles::HillThelenType (name, geometry, characteristics, pathModifiers,
                                     emg)
{
    setType();
}

biorbd::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const biorbd::muscles::Muscle &other) :
    biorbd::muscles::HillThelenType (other)
{

}

biorbd::muscles::HillThelenActiveOnlyType::HillThelenActiveOnlyType(
    const std::shared_ptr<biorbd::muscles::Muscle>
    other) :
    biorbd::muscles::HillThelenType(other)
{

}

biorbd::muscles::HillThelenActiveOnlyType
biorbd::muscles::HillThelenActiveOnlyType::DeepCopy() const
{
    biorbd::muscles::HillThelenActiveOnlyType copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::HillThelenActiveOnlyType::DeepCopy(
    const biorbd::muscles::HillThelenActiveOnlyType &other)
{
    biorbd::muscles::HillThelenType::DeepCopy(other);
}

void biorbd::muscles::HillThelenActiveOnlyType::computeFlPE()
{
    *m_FlPE = 0;
}

void biorbd::muscles::HillThelenActiveOnlyType::computeDamping()
{
    *m_damping = 0;
}

void biorbd::muscles::HillThelenActiveOnlyType::setType()
{
    *m_type = biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE;
}
