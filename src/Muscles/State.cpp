#define BIORBD_API_EXPORTS
#include "Muscles/State.h"

#include "Utils/Error.h"

biorbd::muscles::State::State(
    const biorbd::utils::Scalar& excitation,
    const biorbd::utils::Scalar& activation) :
    m_stateType(std::make_shared<biorbd::muscles::STATE_TYPE>()),
    m_excitation(std::make_shared<biorbd::utils::Scalar>(excitation)),
    m_excitationNorm(std::make_shared<biorbd::utils::Scalar>(0)),
    m_activation(std::make_shared<biorbd::utils::Scalar>(activation))
{
    setType();
}

biorbd::muscles::State::State(
    const biorbd::muscles::State &other) :
    m_stateType(other.m_stateType),
    m_excitation(other.m_excitation),
    m_excitationNorm(std::make_shared<biorbd::utils::Scalar>(0)),
    m_activation(other.m_activation)
{

}

biorbd::muscles::State::~State()
{
    //dtor
}

biorbd::muscles::State biorbd::muscles::State::DeepCopy() const
{
    biorbd::muscles::State copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::State::DeepCopy(const biorbd::muscles::State &other)
{
    *m_stateType = *other.m_stateType;
    *m_excitation = *other.m_excitation;
    *m_excitationNorm = *other.m_excitationNorm;
    *m_activation = *other.m_activation;
}

void biorbd::muscles::State::setExcitation(
    const biorbd::utils::Scalar& val,
    bool turnOffWarnings)
{

#ifdef BIORBD_USE_CASADI_MATH
    *m_excitation = val;
#else
    if (val<0) {
        if (!turnOffWarnings) {
            biorbd::utils::Error::warning(
                0, "Excitation can't be lower than 0, 0 is used then");
        }
        *m_excitation = 0;
    } else {
        *m_excitation = val;
    }
#endif
}

const biorbd::utils::Scalar& biorbd::muscles::State::excitation() const
{
    return *m_excitation;
}

const biorbd::utils::Scalar& biorbd::muscles::State::normalizeExcitation(
    const biorbd::muscles::State &emgMax,
    bool turnOffWarnings)
{

#ifndef BIORBD_USE_CASADI_MATH
    if (!turnOffWarnings) {
        biorbd::utils::Error::warning(
            *m_excitation < emgMax.excitation(),
            "Excitation is higher than maximal excitation.");
    }
#endif
    *m_excitationNorm = *m_excitation / emgMax.excitation();

    return *m_excitationNorm;
}

void biorbd::muscles::State::setExcitationNorm(
    const biorbd::utils::Scalar& val)
{
    *m_excitationNorm = val;
}

const biorbd::utils::Scalar& biorbd::muscles::State::excitationNorm() const
{
    return *m_excitationNorm;
}

void biorbd::muscles::State::setActivation(
    const biorbd::utils::Scalar& val,
    bool turnOffWarnings)
{
#ifdef BIORBD_USE_CASADI_MATH
    *m_activation = val;
#else
    if (val <= 0) {
        if (!turnOffWarnings) {
            biorbd::utils::Error::warning(
                0, "Activation is " + biorbd::utils::String::to_string(val) +
                " but can't be lower than 0, 0 is used then");
        }
        *m_activation = 0;
    } else if (val >= 1) {
        if (!turnOffWarnings) {
            biorbd::utils::Error::warning(
                0, "Activation " + biorbd::utils::String::to_string(val) +
                " but can't be higher than 1, 1 is used then");
        }
        *m_activation = 1;
    } else {
        *m_activation = val;
    }
#endif
}

const biorbd::utils::Scalar& biorbd::muscles::State::activation() const
{
    return *m_activation;
}

biorbd::muscles::STATE_TYPE biorbd::muscles::State::type() const
{
    return *m_stateType;
}

void biorbd::muscles::State::setType()
{
    *m_stateType = biorbd::muscles::STATE_TYPE::SIMPLE_STATE;
}
