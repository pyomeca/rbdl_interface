#define BIORBD_API_EXPORTS
#include "Actuators/Actuator.h"

#include "Utils/Error.h"
#include "Utils/String.h"

biorbd::actuator::Actuator::Actuator() :
    m_type(std::make_shared<biorbd::actuator::TYPE>
           (biorbd::actuator::TYPE::NO_TYPE)),
    m_direction(std::make_shared<int>(0)),
    m_jointName(std::make_shared<biorbd::utils::String>("")),
    m_dofIdx(std::make_shared<unsigned int>(-1))
{

}

biorbd::actuator::Actuator::Actuator(
    const biorbd::actuator::Actuator &other) :
    m_type(other.m_type),
    m_direction(other.m_direction),
    m_jointName(other.m_jointName),
    m_dofIdx(other.m_dofIdx)
{

}

biorbd::actuator::Actuator::Actuator(
    int direction,
    unsigned int dofIdx) :
    m_type(std::make_shared<biorbd::actuator::TYPE>
           (biorbd::actuator::TYPE::NO_TYPE)),
    m_direction(std::make_shared<int>(direction)),
    m_jointName(std::make_shared<biorbd::utils::String>("")),
    m_dofIdx(std::make_shared<unsigned int>(dofIdx))
{

}

biorbd::actuator::Actuator::Actuator(
    int direction,
    unsigned int dofIdx,
    const biorbd::utils::String &jointName) :
    m_type(std::make_shared<biorbd::actuator::TYPE>
           (biorbd::actuator::TYPE::NO_TYPE)),
    m_direction(std::make_shared<int>(direction)),
    m_jointName(std::make_shared<biorbd::utils::String>(jointName)),
    m_dofIdx(std::make_shared<unsigned int>(dofIdx))
{
    biorbd::utils::Error::check(*m_direction==-1 || *m_direction==1,
                                "Direction should be -1 or 1");
}

biorbd::actuator::Actuator::~Actuator()
{

}

void biorbd::actuator::Actuator::DeepCopy(const biorbd::actuator::Actuator
        &other)
{
    *m_type = *other.m_type;
    *m_direction = *other.m_direction;
    *m_jointName = *other.m_jointName;
    *m_dofIdx = *other.m_dofIdx;
}

unsigned int biorbd::actuator::Actuator::index() const
{
    return *m_dofIdx;
}

int biorbd::actuator::Actuator::direction() const
{
    return *m_direction;
}

biorbd::actuator::TYPE biorbd::actuator::Actuator::type() const
{
    return *m_type;
}
