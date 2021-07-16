#define BIORBD_API_EXPORTS
#include "Muscles/FatigueModel.h"

#include "Utils/Error.h"
#include "Muscles/Muscle.h"
#include "Muscles/FatigueDynamicStateXia.h"

biorbd::muscles::FatigueModel::FatigueModel(
    biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType)
{
    if (dynamicFatigueType ==
            biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE) {
        m_fatigueState = std::make_shared<biorbd::muscles::FatigueState>();
    } else if (dynamicFatigueType ==
               biorbd::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA) {
        m_fatigueState = std::make_shared<biorbd::muscles::FatigueDynamicStateXia>();
    } else {
        biorbd::utils::Error::raise("Wrong muscle fatigue type");
    }
}

biorbd::muscles::FatigueModel::FatigueModel(
    const biorbd::muscles::FatigueModel &other) :
    m_fatigueState(other.m_fatigueState)
{

}

biorbd::muscles::FatigueModel::FatigueModel(
    const std::shared_ptr<biorbd::muscles::FatigueModel> other) :
    m_fatigueState(other->m_fatigueState)
{

}

biorbd::muscles::FatigueModel::~FatigueModel()
{

}

void biorbd::muscles::FatigueModel::DeepCopy(const biorbd::muscles::FatigueModel
        &other)
{
    *m_fatigueState = other.m_fatigueState->DeepCopy();
}

#ifndef BIORBD_USE_CASADI_MATH
void biorbd::muscles::FatigueModel::setFatigueState(
    const biorbd::utils::Scalar& active,
    const biorbd::utils::Scalar& fatigued,
    const biorbd::utils::Scalar& resting)
{
    m_fatigueState->setState(active, fatigued, resting);
}
#endif

biorbd::muscles::FatigueState& biorbd::muscles::FatigueModel::fatigueState()
{
    return *m_fatigueState;
}

const biorbd::muscles::FatigueState&
biorbd::muscles::FatigueModel::fatigueState() const
{
    return *m_fatigueState;
}

void biorbd::muscles::FatigueModel::computeTimeDerivativeState(
    const biorbd::muscles::StateDynamics &emg)
{
    if (std::dynamic_pointer_cast<biorbd::muscles::FatigueDynamicState>
            (m_fatigueState)) {
        biorbd::muscles::Muscle* muscle = dynamic_cast<biorbd::muscles::Muscle*>(this);
        if (muscle) {
            std::static_pointer_cast<biorbd::muscles::FatigueDynamicState>
            (m_fatigueState)->timeDerivativeState(emg,
                                                  muscle->characteristics());
        } else {
            biorbd::utils::Error::raise("biorbd::muscles::FatigueModel should be a biorbd::muscles::Muscle");
        }
    } else {
        biorbd::utils::Error::raise("Type cannot be fatigued");
    }
}

