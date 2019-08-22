#define BIORBD_API_EXPORTS
#include "Muscles/State.h"

biorbd::muscles::State::State(
        double e,
        double a) :
    m_excitation(e),
    m_activation(a)
{
}

biorbd::muscles::State::~State()
{
    //dtor
}

void biorbd::muscles::State::setExcitation(double val) {
    if (m_excitation<=0)
        m_excitation = 0;
    else
        m_excitation = val;
}

void biorbd::muscles::State::setActivation(double val){
    if (m_activation<=0)
        m_activation = 0;
    else if (m_activation>=1)
        m_activation = 1;
    else
        m_activation = val;
}

double biorbd::muscles::State::excitation() const
{
    return m_excitation;
}

double biorbd::muscles::State::activation() const
{
    return m_activation;
}