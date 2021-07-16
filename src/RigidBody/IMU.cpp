#define BIORBD_API_EXPORTS
#include "RigidBody/IMU.h"

#include "Utils/String.h"

biorbd::rigidbody::IMU::IMU(
    bool isTechnical,
    bool isAnatomical) :
    biorbd::utils::RotoTransNode(),
    m_technical(std::make_shared<bool>(isTechnical)),
    m_anatomical(std::make_shared<bool>(isAnatomical))
{

}

biorbd::rigidbody::IMU::IMU(
    const biorbd::utils::RotoTransNode &RotoTrans,
    bool isTechnical,
    bool isAnatomical) :
    biorbd::utils::RotoTransNode(RotoTrans),
    m_technical(std::make_shared<bool>(isTechnical)),
    m_anatomical(std::make_shared<bool>(isAnatomical))
{

}

#ifdef BIORBD_USE_CASADI_MATH

biorbd::rigidbody::IMU::IMU(
    const biorbd::rigidbody::IMU &imu) :
    biorbd::utils::RotoTransNode (imu),
    m_technical(std::make_shared<bool>(*imu.m_technical)),
    m_anatomical(std::make_shared<bool>(*imu.m_anatomical))
{

}

biorbd::rigidbody::IMU biorbd::rigidbody::IMU::operator*(
    const biorbd::rigidbody::IMU &other) const
{
    return biorbd::rigidbody::IMU(
               biorbd::utils::RotoTransNode(
                   this->biorbd::utils::RotoTransNode::operator*(other),
                   this->biorbd::utils::Node::name(),
                   this->parent()),
               this->isTechnical() && other.isTechnical(),
               this->isAnatomical() && other.isAnatomical());
}

#endif

biorbd::rigidbody::IMU biorbd::rigidbody::IMU::DeepCopy() const
{
    biorbd::rigidbody::IMU copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::IMU::DeepCopy(const IMU &other)
{
    biorbd::utils::RotoTransNode::DeepCopy(other);
    *m_technical = *other.m_technical;
    *m_anatomical = *other.m_anatomical;
}

bool biorbd::rigidbody::IMU::isAnatomical() const
{
    return *m_anatomical;
}

bool biorbd::rigidbody::IMU::isTechnical() const
{
    return *m_technical;
}
