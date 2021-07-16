#define BIORBD_API_EXPORTS
#include "Muscles/StateDynamicsDeGroote.h"

#include <math.h>
#include "Utils/Error.h"
#include "Utils/String.h"
#include "Muscles/Characteristics.h"

biorbd::muscles::StateDynamicsDeGroote::StateDynamicsDeGroote(
    const biorbd::utils::Scalar& excitation,
    const biorbd::utils::Scalar& activation) :
    biorbd::muscles::StateDynamics(excitation,activation)
{
    setType();
}

biorbd::muscles::StateDynamicsDeGroote::StateDynamicsDeGroote(
    const biorbd::muscles::StateDynamicsDeGroote &other) :
    biorbd::muscles::StateDynamics(other)
{

}

biorbd::muscles::StateDynamicsDeGroote
biorbd::muscles::StateDynamicsDeGroote::DeepCopy() const
{
    biorbd::muscles::StateDynamicsDeGroote copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::StateDynamicsDeGroote::DeepCopy(const
        biorbd::muscles::StateDynamicsDeGroote &other)
{
    biorbd::muscles::StateDynamics::DeepCopy(other);
}

const biorbd::utils::Scalar&
biorbd::muscles::StateDynamicsDeGroote::timeDerivativeActivation(
    const biorbd::muscles::Characteristics &characteristics,
    bool alreadyNormalized)
{
    // From DeGroote https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5043004/

    // Get activation and excitation and make difference
    biorbd::utils::Scalar diff; //(e - a)
    biorbd::utils::Scalar f;    //activation dynamics
    //std::tanh;

    diff = *m_excitation - *m_activation;
    f = 0.5 * tanh(0.1*diff);

    biorbd::utils::Scalar denom_activation = characteristics.torqueActivation()
            * (0.5+1.5* *m_activation);
    biorbd::utils::Scalar denom_deactivation =
        characteristics.torqueDeactivation() / (0.5+1.5* *m_activation);

    *m_activationDot = (((f+0.5)/denom_activation)+((-f + 0.5)/denom_deactivation))
                       *diff;
    return *m_activationDot;
}

void biorbd::muscles::StateDynamicsDeGroote::setType()
{
    *m_stateType = biorbd::muscles::STATE_TYPE::DE_GROOTE;
}
