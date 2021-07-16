#define BIORBD_API_EXPORTS
#include "Muscles/StateDynamics.h"

#include "Utils/Error.h"
#include "Utils/String.h"
#include "Muscles/Characteristics.h"

biorbd::muscles::StateDynamics::StateDynamics(
    const biorbd::utils::Scalar& excitation,
    const biorbd::utils::Scalar& activation) :
    biorbd::muscles::State(excitation,activation),
    m_previousExcitation(std::make_shared<biorbd::utils::Scalar>(0)),
    m_previousActivation(std::make_shared<biorbd::utils::Scalar>(0)),
    m_activationDot(std::make_shared<biorbd::utils::Scalar>(0))
{
    setType();
}

biorbd::muscles::StateDynamics::StateDynamics(
    const biorbd::muscles::State &other) :
    biorbd::muscles::State(other)
{
    const auto& state = dynamic_cast<const biorbd::muscles::StateDynamics&>(other);
    m_previousExcitation = state.m_previousExcitation;
    m_previousActivation = state.m_previousActivation;
    m_activationDot = state.m_activationDot;
}

biorbd::muscles::StateDynamics::~StateDynamics()
{
    //dtor
}

biorbd::muscles::StateDynamics biorbd::muscles::StateDynamics::DeepCopy() const
{
    biorbd::muscles::StateDynamics copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::StateDynamics::DeepCopy(const
        biorbd::muscles::StateDynamics &other)
{
    biorbd::muscles::State::DeepCopy(other);
    *m_excitationNorm = *other.m_excitationNorm;
    *m_previousExcitation = *other.m_previousExcitation;
    *m_previousActivation = *other.m_previousActivation;
    *m_activationDot = *other.m_activationDot;
}


const biorbd::utils::Scalar&
biorbd::muscles::StateDynamics::timeDerivativeActivation(
    const biorbd::muscles::State& emg,
    const biorbd::muscles::Characteristics& characteristics,
    bool alreadyNormalized)
{
    return timeDerivativeActivation(emg.excitation(), emg.activation(),
                                    characteristics, alreadyNormalized);
}


const biorbd::utils::Scalar&
biorbd::muscles::StateDynamics::timeDerivativeActivation(
    const biorbd::utils::Scalar& excitation,
    const biorbd::utils::Scalar& activation,
    const biorbd::muscles::Characteristics &characteristics,
    bool alreadyNormalized)
{
    setExcitation(excitation);
    setActivation(activation);
    return timeDerivativeActivation(characteristics, alreadyNormalized);
}

const biorbd::utils::Scalar&
biorbd::muscles::StateDynamics::timeDerivativeActivation(
    const biorbd::muscles::Characteristics &characteristics,
    bool alreadyNormalized)
{
    // Implémentation de la fonction da/dt = (u-a)/GeneralizedTorque(u,a)
    // ou GeneralizedTorque(u,a) = t_act(0.5+1.5*a) is u>a et GeneralizedTorque(u,a)=t_deact(0.5+1.5*a) sinon
#ifdef BIORBD_USE_CASADI_MATH
    *m_activation = casadi::MX::if_else(
                        casadi::MX::lt(*m_activation, characteristics.minActivation()),
                        characteristics.minActivation(), *m_activation);
    *m_excitation = casadi::MX::if_else(
                        casadi::MX::lt(*m_excitation, characteristics.minActivation()),
                        characteristics.minActivation(), *m_excitation);
#else
    if (*m_activation < characteristics.minActivation()) {
        *m_activation = characteristics.minActivation();
    }

    if (*m_excitation < characteristics.minActivation()) {
        *m_excitation = characteristics.minActivation();
    }
#endif



// see doi:10.1016/j.humov.2011.08.006
// see doi:10.1016/S0021-9290(03)00010-1


// http://simtk-confluence.stanford.edu:8080/display/OpenSim/First-Order+Activation+Dynamics

    biorbd::utils::Scalar num;
    if (alreadyNormalized) {
        num = *m_excitation - *m_activation;    // numérateur
    } else {
        num = normalizeExcitation(characteristics.stateMax())-
              *m_activation;    // numérateur
    }

    biorbd::utils::Scalar denom; // dénominateur
#ifdef BIORBD_USE_CASADI_MATH
    denom = casadi::MX::if_else(
                casadi::MX::gt(num, 0),
                characteristics.torqueActivation()   * (0.5+1.5* *m_activation),
                characteristics.torqueDeactivation() / (0.5+1.5* *m_activation));
#else
    if (num>0) {
        denom = characteristics.torqueActivation()   * (0.5+1.5* *m_activation);
    } else {
        denom = characteristics.torqueDeactivation() / (0.5+1.5* *m_activation);
    }
#endif
    *m_activationDot = num/denom;

    return *m_activationDot;
}

const biorbd::utils::Scalar&
biorbd::muscles::StateDynamics::timeDerivativeActivation()
{
    return *m_activationDot;
}

void biorbd::muscles::StateDynamics::setExcitation(
    const biorbd::utils::Scalar& val,
    bool turnOffWarnings)
{
    *m_previousExcitation = *m_excitation;
    biorbd::muscles::State::setExcitation(val, turnOffWarnings);
}

const biorbd::utils::Scalar&
biorbd::muscles::StateDynamics::previousExcitation() const
{
    return *m_previousExcitation;
}

void biorbd::muscles::StateDynamics::setActivation(
    const biorbd::utils::Scalar& val,
    bool turnOffWarnings)
{
    *m_previousActivation = *m_activation;
    biorbd::muscles::State::setActivation(val, turnOffWarnings);
}

const biorbd::utils::Scalar&
biorbd::muscles::StateDynamics::previousActivation() const
{
    return *m_previousActivation;
}

void biorbd::muscles::StateDynamics::setType()
{
    *m_stateType = biorbd::muscles::STATE_TYPE::DYNAMIC;
}
