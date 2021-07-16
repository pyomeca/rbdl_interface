#define BIORBD_API_EXPORTS
#include "Muscles/WrappingObject.h"

#include "Utils/String.h"
#include "Utils/RotoTrans.h"

biorbd::muscles::WrappingObject::WrappingObject() :
    biorbd::utils::Vector3d (),
    m_RT(std::make_shared<biorbd::utils::RotoTrans>())
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::WRAPPING_OBJECT;
}

biorbd::muscles::WrappingObject::WrappingObject(
    const biorbd::utils::Scalar& x,
    const biorbd::utils::Scalar& y,
    const biorbd::utils::Scalar& z) :
    biorbd::utils::Vector3d(x, y, z),
    m_RT(std::make_shared<biorbd::utils::RotoTrans>())
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::WRAPPING_OBJECT;
}

biorbd::muscles::WrappingObject::WrappingObject(
    const biorbd::utils::Scalar& x,
    const biorbd::utils::Scalar& y,
    const biorbd::utils::Scalar& z,
    const biorbd::utils::String &name,
    const biorbd::utils::String &parentName) :
    biorbd::utils::Vector3d(x, y, z, name, parentName),
    m_RT(std::make_shared<biorbd::utils::RotoTrans>())
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::WRAPPING_OBJECT;
}

biorbd::muscles::WrappingObject::WrappingObject(
    const biorbd::utils::Vector3d &other) :
    biorbd::utils::Vector3d (other)
{
    try {
        biorbd::muscles::WrappingObject& otherWrap(
            const_cast<biorbd::muscles::WrappingObject&>(
                dynamic_cast<const biorbd::muscles::WrappingObject&>(other)));
        m_RT = otherWrap.m_RT;
    } catch(const std::bad_cast& e) {
        m_RT = std::make_shared<biorbd::utils::RotoTrans>();
    }
}

biorbd::muscles::WrappingObject::WrappingObject(
    const biorbd::utils::Vector3d &other,
    const biorbd::utils::String &name,
    const biorbd::utils::String &parentName) :
    biorbd::utils::Vector3d (other, name, parentName),
    m_RT(std::make_shared<biorbd::utils::RotoTrans>())
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::WRAPPING_OBJECT;
}

void biorbd::muscles::WrappingObject::DeepCopy(
    const biorbd::muscles::WrappingObject &other)
{
    biorbd::utils::Vector3d::DeepCopy(other);
    *m_RT = *other.m_RT;
}

const biorbd::utils::RotoTrans &biorbd::muscles::WrappingObject::RT() const
{
    return *m_RT;
}
