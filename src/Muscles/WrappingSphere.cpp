#define BIORBD_API_EXPORTS
#include "Muscles/WrappingSphere.h"

#include "Utils/String.h"
#include "Utils/RotoTrans.h"

biorbd::muscles::WrappingSphere::WrappingSphere() :
    biorbd::muscles::WrappingObject (),
    m_dia(std::make_shared<biorbd::utils::Scalar>(0))
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::WRAPPING_SPHERE;
}

biorbd::muscles::WrappingSphere::WrappingSphere(
    const biorbd::utils::Scalar& x,
    const biorbd::utils::Scalar& y,
    const biorbd::utils::Scalar& z,
    const biorbd::utils::Scalar& diameter) :
    biorbd::muscles::WrappingObject (x, y, z),
    m_dia(std::make_shared<biorbd::utils::Scalar>(diameter))

{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::WRAPPING_SPHERE;
}

biorbd::muscles::WrappingSphere::WrappingSphere(
    const biorbd::utils::Scalar& x,
    const biorbd::utils::Scalar& y,
    const biorbd::utils::Scalar& z,
    const biorbd::utils::Scalar& diameter,
    const biorbd::utils::String &name,
    const biorbd::utils::String &parentName) :
    biorbd::muscles::WrappingObject (x, y, z, name, parentName),
    m_dia(std::make_shared<biorbd::utils::Scalar>(diameter))
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::WRAPPING_SPHERE;
}

biorbd::muscles::WrappingSphere::WrappingSphere(
    const biorbd::utils::Vector3d &v,
    const biorbd::utils::Scalar& diameter) :
    biorbd::muscles::WrappingObject(v),
    m_dia(std::make_shared<biorbd::utils::Scalar>(diameter))
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::WRAPPING_SPHERE;
}

biorbd::muscles::WrappingSphere biorbd::muscles::WrappingSphere::DeepCopy()
const
{
    biorbd::muscles::WrappingSphere copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::WrappingSphere::DeepCopy(const
        biorbd::muscles::WrappingSphere &other)
{
    biorbd::muscles::WrappingObject::DeepCopy(other);
    *m_dia = *other.m_dia;
}

const biorbd::utils::RotoTrans& biorbd::muscles::WrappingSphere::RT(
    biorbd::rigidbody::Joints &,
    const biorbd::rigidbody::GeneralizedCoordinates &,
    bool)
{
    return *m_RT;
}

void biorbd::muscles::WrappingSphere::setDiameter(
    const biorbd::utils::Scalar& val)
{
    *m_dia = val;
}

const biorbd::utils::Scalar& biorbd::muscles::WrappingSphere::diameter() const
{
    return *m_dia;
}
