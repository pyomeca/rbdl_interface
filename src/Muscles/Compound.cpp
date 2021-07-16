#define BIORBD_API_EXPORTS
#include "Muscles/Compound.h"

#include "Utils/String.h"
#include "Utils/Vector3d.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "Muscles/Characteristics.h"
#include "Muscles/PathModifiers.h"
#include "Muscles/State.h"

biorbd::muscles::Compound::Compound() :
    m_name(std::make_shared<biorbd::utils::String>("")),
    m_type(std::make_shared<biorbd::muscles::MUSCLE_TYPE>
           (biorbd::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_pathChanger(std::make_shared<biorbd::muscles::PathModifiers>()),
    m_force(std::make_shared<biorbd::utils::Scalar>(0))
{

}

biorbd::muscles::Compound::Compound(const biorbd::utils::String &name) :
    m_name(std::make_shared<biorbd::utils::String>(name)),
    m_type(std::make_shared<biorbd::muscles::MUSCLE_TYPE>
           (biorbd::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_pathChanger(std::make_shared<biorbd::muscles::PathModifiers>()),
    m_force(std::make_shared<biorbd::utils::Scalar>(0))
{

}

biorbd::muscles::Compound::Compound(
    const biorbd::utils::String &name,
    const biorbd::muscles::PathModifiers &pathModifiers) :
    m_name(std::make_shared<biorbd::utils::String>(name)),
    m_type(std::make_shared<biorbd::muscles::MUSCLE_TYPE>
           (biorbd::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE)),
    m_pathChanger(std::make_shared<biorbd::muscles::PathModifiers>(pathModifiers)),
    m_force(std::make_shared<biorbd::utils::Scalar>(0))
{

}

biorbd::muscles::Compound::Compound(
    const biorbd::muscles::Compound &other) :
    m_name(other.m_name),
    m_type(other.m_type),
    m_pathChanger(other.m_pathChanger),
    m_force(other.m_force)
{

}

biorbd::muscles::Compound::Compound(
    std::shared_ptr<biorbd::muscles::Compound> other) :
    m_name(other->m_name),
    m_type(other->m_type),
    m_pathChanger(other->m_pathChanger),
    m_force(other->m_force)
{

}

biorbd::muscles::Compound::~Compound()
{

}

void biorbd::muscles::Compound::DeepCopy(const biorbd::muscles::Compound &other)
{
    *m_name = *other.m_name;
    *m_type = *other.m_type;
    *m_pathChanger = other.m_pathChanger->DeepCopy();
    *m_force = *other.m_force;
}

const biorbd::utils::String &biorbd::muscles::Compound::name() const
{
    return *m_name;
}

void biorbd::muscles::Compound::setName(const biorbd::utils::String &name)
{
    *m_name = name;
}

biorbd::muscles::MUSCLE_TYPE biorbd::muscles::Compound::type() const
{
    return *m_type;
}

const biorbd::muscles::PathModifiers &biorbd::muscles::Compound::pathModifier()
{
    return *m_pathChanger;
}

void biorbd::muscles::Compound::addPathObject(biorbd::utils::Vector3d &wrap)
{
    m_pathChanger->addPathChanger(wrap);
}

const biorbd::utils::Scalar& biorbd::muscles::Compound::force()
{
    return *m_force;
}
