#define BIORBD_API_EXPORTS
#include "Muscles/FatigueDynamicStateXia.h"

#include <cmath>
#include "Utils/Error.h"
#include "Muscles/Characteristics.h"
#include "Muscles/FatigueParameters.h"
#include "Muscles/StateDynamics.h"

biorbd::muscles::FatigueDynamicStateXia::FatigueDynamicStateXia(
    const biorbd::utils::Scalar& active,
    const biorbd::utils::Scalar& fatigued,
    const biorbd::utils::Scalar& resting) :
    biorbd::muscles::FatigueDynamicState(active,fatigued,resting)
{
    setType();
}

biorbd::muscles::FatigueDynamicStateXia::FatigueDynamicStateXia(
    const std::shared_ptr<biorbd::muscles::FatigueState> other):
    biorbd::muscles::FatigueDynamicState(other)
{

}

biorbd::muscles::FatigueDynamicStateXia
biorbd::muscles::FatigueDynamicStateXia::DeepCopy() const
{
    biorbd::muscles::FatigueDynamicStateXia copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::FatigueDynamicStateXia::DeepCopy(const
        biorbd::muscles::FatigueDynamicStateXia &other)
{
    biorbd::muscles::FatigueDynamicState::DeepCopy(other);
}

void biorbd::muscles::FatigueDynamicStateXia::timeDerivativeState(
    const biorbd::muscles::StateDynamics &emg,
    const biorbd::muscles::Characteristics &characteristics)
{
#ifdef BIORBD_USE_CASADI_MATH
    biorbd::utils::Error::raise("timeDerivativeState for FatigueDynamicStateXia"
                                " is not implemented yet");
#else
    // Getting the command
    biorbd::utils::Scalar targetCommand(emg.activation());
    biorbd::utils::Scalar command(0);
    if (*m_activeFibers < targetCommand) {
        if (*m_restingFibers > targetCommand - *m_activeFibers) {
            command = characteristics.fatigueParameters().developFactor()*
                      (targetCommand - *m_activeFibers);
        } else {
            command = characteristics.fatigueParameters().developFactor()* *m_restingFibers;
        }
    } else {
        command = characteristics.fatigueParameters().recoveryFactor()*
                  (targetCommand - *m_activeFibers);
    }

    // Applying the command to the fibers
    *m_activeFibersDot = command - characteristics.fatigueParameters().fatigueRate()
                         * *m_activeFibers;
    *m_restingFibersDot = -command +
                          characteristics.fatigueParameters().recoveryRate()* *m_fatiguedFibers;
    *m_fatiguedFibersDot = characteristics.fatigueParameters().fatigueRate()*
                           *m_activeFibers -
                           characteristics.fatigueParameters().recoveryRate()* *m_fatiguedFibers;

    biorbd::utils::Error::check(
        fabs(*m_activeFibersDot + *m_restingFibersDot + *m_fatiguedFibersDot) <= 1e-7,
        "Sum of time derivates of fatigue states must be equal to 0");
#endif
}

void biorbd::muscles::FatigueDynamicStateXia::setType()
{
    *m_type =biorbd::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA;
}
