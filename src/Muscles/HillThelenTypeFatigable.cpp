#define BIORBD_API_EXPORTS
#include "Muscles/HillThelenTypeFatigable.h"

#include "Utils/String.h"
#include "Muscles/FatigueState.h"

biorbd::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable() :
    biorbd::muscles::HillThelenType(),
    biorbd::muscles::FatigueModel (
        biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE)
{
    setType();
}

biorbd::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const biorbd::utils::String &name,
    const biorbd::muscles::Geometry &geometry,
    const biorbd::muscles::Characteristics &characteristics,
    biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    biorbd::muscles::HillThelenType(name, geometry, characteristics),
    biorbd::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

biorbd::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const biorbd::utils::String &name,
    const biorbd::muscles::Geometry &geometry,
    const biorbd::muscles::Characteristics &characteristics,
    const biorbd::muscles::State &emg,
    biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    biorbd::muscles::HillThelenType(name, geometry, characteristics, emg),
    biorbd::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

biorbd::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const biorbd::utils::String &name,
    const biorbd::muscles::Geometry &geometry,
    const biorbd::muscles::Characteristics &characteristics,
    const biorbd::muscles::PathModifiers &pathModifiers,
    biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    biorbd::muscles::HillThelenType(name, geometry, characteristics, pathModifiers),
    biorbd::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

biorbd::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const biorbd::utils::String& name,
    const biorbd::muscles::Geometry& geometry,
    const biorbd::muscles::Characteristics& characteristics,
    const biorbd::muscles::PathModifiers& pathModifiers,
    const biorbd::muscles::State& emg,
    biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType) :
    biorbd::muscles::HillThelenType(name, geometry, characteristics, pathModifiers,
                                    emg),
    biorbd::muscles::FatigueModel (dynamicFatigueType)
{
    setType();
}

biorbd::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const biorbd::muscles::Muscle &other) :
    biorbd::muscles::HillThelenType (other),
    biorbd::muscles::FatigueModel (
        dynamic_cast<const biorbd::muscles::FatigueModel&>(other))
{

}

biorbd::muscles::HillThelenTypeFatigable::HillThelenTypeFatigable(
    const std::shared_ptr<biorbd::muscles::Muscle> other) :
    biorbd::muscles::HillThelenType (other),
    biorbd::muscles::FatigueModel (
        std::dynamic_pointer_cast<biorbd::muscles::FatigueModel>(other))
{

}

biorbd::muscles::HillThelenTypeFatigable
biorbd::muscles::HillThelenTypeFatigable::DeepCopy() const
{
    biorbd::muscles::HillThelenTypeFatigable copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::HillThelenTypeFatigable::DeepCopy(
    const biorbd::muscles::HillThelenTypeFatigable &other)
{
    biorbd::muscles::HillThelenType::DeepCopy(other);
    biorbd::muscles::FatigueModel::DeepCopy(other);
}

void biorbd::muscles::HillThelenTypeFatigable::computeFlCE(
    const biorbd::muscles::State &emg)
{
    biorbd::muscles::HillThelenType::computeFlCE(emg);
    // Do something with m_FlCE and m_characteristics.fatigueParameters
    *m_FlCE *= m_fatigueState->activeFibers();
}

void biorbd::muscles::HillThelenTypeFatigable::setType()
{
    *m_type = biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE;
}
