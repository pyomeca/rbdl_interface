#define BIORBD_API_EXPORTS
#include "Muscles/PathModifiers.h"

#include "Utils/Error.h"
#include "Utils/Vector3d.h"
#include "Muscles/ViaPoint.h"
#include "Muscles/WrappingSphere.h"
#include "Muscles/WrappingHalfCylinder.h"

biorbd::muscles::PathModifiers::PathModifiers() :
    m_obj(std::make_shared<std::vector<std::shared_ptr<biorbd::utils::Vector3d>>>()),
    m_nbWraps(std::make_shared<unsigned int>(0)),
    m_nbVia(std::make_shared<unsigned int>(0)),
    m_totalObjects(std::make_shared<unsigned int>(0))
{

}

biorbd::muscles::PathModifiers biorbd::muscles::PathModifiers::DeepCopy() const
{
    biorbd::muscles::PathModifiers copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::PathModifiers::DeepCopy(const
        biorbd::muscles::PathModifiers &other)
{
    m_obj->resize(other.m_obj->size());
    for (unsigned int i=0; i<other.m_obj->size(); ++i) {
        (*m_obj)[i] = std::make_shared<biorbd::utils::Vector3d>();
        *(*m_obj)[i] = (*other.m_obj)[i]->DeepCopy();
    }
    *m_nbWraps = *other.m_nbWraps;
    *m_nbVia = *other.m_nbVia;
    *m_totalObjects = *other.m_totalObjects;
}

// Private method to assing values
void biorbd::muscles::PathModifiers::addPathChanger(
    biorbd::utils::Vector3d &object)
{

    // Add a muscle to the pool of muscle depending on type
    if (object.typeOfNode() == biorbd::utils::NODE_TYPE::WRAPPING_SPHERE) {
        biorbd::utils::Error::check(*m_nbVia == 0,
                                    "Cannot mix via points and wrapping objects yet");
        m_obj->push_back(std::make_shared<biorbd::muscles::WrappingSphere>(
                             static_cast<biorbd::muscles::WrappingSphere&> (object)));
        ++*m_nbWraps;
    } else if (object.typeOfNode() ==
               biorbd::utils::NODE_TYPE::WRAPPING_HALF_CYLINDER) {
        biorbd::utils::Error::check(*m_nbVia == 0,
                                    "Cannot mix via points and wrapping objects yet");
        m_obj->push_back(std::make_shared<biorbd::muscles::WrappingHalfCylinder>(
                             dynamic_cast <biorbd::muscles::WrappingHalfCylinder&> (object)));
        ++*m_nbWraps;
    } else if (object.typeOfNode() == biorbd::utils::NODE_TYPE::VIA_POINT) {
        biorbd::utils::Error::check(*m_nbWraps == 0,
                                    "Cannot mix via points and wrapping objects yet");
        m_obj->push_back(std::make_shared<biorbd::muscles::ViaPoint>(
                             dynamic_cast <biorbd::muscles::ViaPoint&> (object)));
        ++*m_nbVia;
    } else {
        biorbd::utils::Error::raise("Wrapping type not found");
    }
    ++*m_totalObjects;
}

unsigned int biorbd::muscles::PathModifiers::nbWraps() const
{
    return *m_nbWraps;
}

unsigned int biorbd::muscles::PathModifiers::nbVia() const
{
    return *m_nbVia;
}

unsigned int biorbd::muscles::PathModifiers::nbObjects() const
{
    return *m_totalObjects;
}

biorbd::utils::Vector3d& biorbd::muscles::PathModifiers::object(
    unsigned int idx)
{
    biorbd::utils::Error::check(idx<nbObjects(),
                                "Idx asked is higher than number of wrapping objects");
    return *(*m_obj)[idx];
}


const biorbd::utils::Vector3d& biorbd::muscles::PathModifiers::object(
    unsigned int idx) const
{
    biorbd::utils::Error::check(idx<nbObjects(),
                                "Idx asked is higher than number of wrapping objects");
    return *(*m_obj)[idx];
}




