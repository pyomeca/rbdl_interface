#define BIORBD_API_EXPORTS
#include "Muscles/FatigueParameters.h"

biorbd::muscles::FatigueParameters::FatigueParameters(
    const biorbd::utils::Scalar& _fatigueRate,
    const biorbd::utils::Scalar& _recoveryRate,
    const biorbd::utils::Scalar& _developFactor,
    const biorbd::utils::Scalar& recoveryFactor):
    m_fatigueRate(std::make_shared<biorbd::utils::Scalar>(_fatigueRate)),
    m_recoveryRate(std::make_shared<biorbd::utils::Scalar>(_recoveryRate)),
    m_developFactor(std::make_shared<biorbd::utils::Scalar>(_developFactor)),
    m_recoveryFactor(std::make_shared<biorbd::utils::Scalar>(recoveryFactor))
{

}

biorbd::muscles::FatigueParameters
biorbd::muscles::FatigueParameters::DeepCopy() const
{
    biorbd::muscles::FatigueParameters copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::FatigueParameters::DeepCopy(const
        biorbd::muscles::FatigueParameters &other)
{
    *m_fatigueRate = *other.m_fatigueRate;
    *m_recoveryRate = *other.m_recoveryRate;
    *m_developFactor = *other.m_developFactor;
    *m_recoveryFactor = *other.m_recoveryFactor;
}

void biorbd::muscles::FatigueParameters::setFatigueRate(
    const biorbd::utils::Scalar& fatigueRate)
{
    *m_fatigueRate = fatigueRate;
}
const biorbd::utils::Scalar& biorbd::muscles::FatigueParameters::fatigueRate()
const
{
    return *m_fatigueRate;
}

void biorbd::muscles::FatigueParameters::setRecoveryRate(
    const biorbd::utils::Scalar& recoveryRate)
{
    *m_recoveryRate = recoveryRate;
}
const biorbd::utils::Scalar& biorbd::muscles::FatigueParameters::recoveryRate()
const
{
    return *m_recoveryRate;
}

void biorbd::muscles::FatigueParameters::setDevelopFactor(
    const biorbd::utils::Scalar& developFactor)
{
    *m_developFactor = developFactor;
}
const biorbd::utils::Scalar& biorbd::muscles::FatigueParameters::developFactor()
const
{
    return *m_developFactor;
}

void biorbd::muscles::FatigueParameters::setRecoveryFactor(
    const biorbd::utils::Scalar& recoveryFactor)
{
    *m_recoveryFactor = recoveryFactor;
}
const biorbd::utils::Scalar&
biorbd::muscles::FatigueParameters::recoveryFactor() const
{
    return *m_recoveryFactor;
}
