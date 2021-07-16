#define BIORBD_API_EXPORTS
#include "RigidBody/NodeSegment.h"

#include "Utils/Error.h"

biorbd::rigidbody::NodeSegment::NodeSegment() :
    biorbd::utils::Vector3d(0, 0, 0),
    m_axesRemoved(std::make_shared<std::vector<bool>>(3)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(true)),
    m_anatomical(std::make_shared<bool>(false)),
    m_id(std::make_shared<int>(-1))
{
    setType();
}

biorbd::rigidbody::NodeSegment::NodeSegment(
    const biorbd::utils::Scalar& x,
    const biorbd::utils::Scalar& y,
    const biorbd::utils::Scalar& z) :
    biorbd::utils::Vector3d(x, y, z),
    m_axesRemoved(std::make_shared<std::vector<bool>>(3)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(true)),
    m_anatomical(std::make_shared<bool>(false)),
    m_id(std::make_shared<int>(-1))
{
    setType();
}

biorbd::rigidbody::NodeSegment::NodeSegment(const biorbd::utils::Vector3d
        &other) :
    biorbd::utils::Vector3d(other),
    m_axesRemoved(std::make_shared<std::vector<bool>>(3)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(true)),
    m_anatomical(std::make_shared<bool>(false)),
    m_id(std::make_shared<int>(-1))
{
    setType();
}

biorbd::rigidbody::NodeSegment::NodeSegment(
    const biorbd::utils::Scalar& x,
    const biorbd::utils::Scalar& y,
    const biorbd::utils::Scalar& z,
    const biorbd::utils::String &name,
    const biorbd::utils::String &parentName,
    bool isTechnical,
    bool isAnatomical,
    const biorbd::utils::String &axesToRemove,
    int parentID) :
    biorbd::utils::Vector3d(x, y, z, name, parentName),
    m_axesRemoved(std::make_shared<std::vector<bool>>(3)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(isTechnical)),
    m_anatomical(std::make_shared<bool>(isAnatomical)),
    m_id(std::make_shared<int>(parentID))
{
    addAxesToRemove(axesToRemove);
}

biorbd::rigidbody::NodeSegment::NodeSegment(
    const biorbd::utils::Vector3d &node,
    const biorbd::utils::String &name,
    const biorbd::utils::String &parentName,
    bool isTechnical,
    bool isAnatomical,
    const biorbd::utils::String& axesToRemove, // Axis to remove
    int parentID) :
    biorbd::utils::Vector3d(node, name, parentName),
    m_axesRemoved(std::make_shared<std::vector<bool>>(3)),
    m_nbAxesToRemove(std::make_shared<int>(0)),
    m_technical(std::make_shared<bool>(isTechnical)),
    m_anatomical(std::make_shared<bool>(isAnatomical)),
    m_id(std::make_shared<int>(parentID))
{
    addAxesToRemove(axesToRemove);
    //ctor
}

biorbd::rigidbody::NodeSegment biorbd::rigidbody::NodeSegment::DeepCopy() const
{
    biorbd::rigidbody::NodeSegment copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::NodeSegment::DeepCopy(const
        biorbd::rigidbody::NodeSegment&other)
{
    biorbd::utils::Node::DeepCopy(other);
    *m_axesRemoved = *other.m_axesRemoved;
    *m_nbAxesToRemove = *other.m_nbAxesToRemove;
    *m_technical = *other.m_technical;
    *m_anatomical = *other.m_anatomical;
    *m_id = *other.m_id;
}


bool biorbd::rigidbody::NodeSegment::isAnatomical() const
{
    return *m_anatomical;
}



bool biorbd::rigidbody::NodeSegment::isTechnical() const
{
    return *m_technical;
}


int biorbd::rigidbody::NodeSegment::parentId() const
{
    return *m_id;
}

biorbd::rigidbody::NodeSegment biorbd::rigidbody::NodeSegment::removeAxes()
const
{
    biorbd::rigidbody::NodeSegment pos(*this);
    for (unsigned int i=0; i<m_axesRemoved->size(); ++i)
        if (isAxisRemoved(i)) {
            pos(i) = 0;
        }
    return pos;
}

bool biorbd::rigidbody::NodeSegment::isAxisRemoved(unsigned int i) const
{
    return (*m_axesRemoved)[i];
}

bool biorbd::rigidbody::NodeSegment::isAxisKept(unsigned int i) const
{
    return !isAxisRemoved(i);
}

int biorbd::rigidbody::NodeSegment::nbAxesToRemove() const
{
    return *m_nbAxesToRemove;
}

void biorbd::rigidbody::NodeSegment::setType()
{
    *m_typeOfNode = biorbd::utils::NODE_TYPE::BONE_POINT;
}

void biorbd::rigidbody::NodeSegment::addAxesToRemove(unsigned int axisNumber)
{
    if (axisNumber>2) {
        biorbd::utils::Error::raise("Axis must be 0 (\"x\"), 1 (\"y\") or 2 (\"z\")");
    }
    (*m_axesRemoved)[axisNumber] = true;
    ++*m_nbAxesToRemove;
}

void biorbd::rigidbody::NodeSegment::addAxesToRemove(const
        biorbd::utils::String& s)
{
    for (unsigned int i=0; i<s.length(); ++i)
        if (!s(i).compare("x")) {
            addAxesToRemove(0);
        } else if (!s(i).compare("y")) {
            addAxesToRemove(1);
        } else if (!s(i).compare("z")) {
            addAxesToRemove(2);
        } else {
            biorbd::utils::Error::raise("Axis must be 0 (\"x\"), 1 (\"y\") or 2 (\"z\")");
        }
}

void biorbd::rigidbody::NodeSegment::addAxesToRemove(const
        std::vector<unsigned int>& axes)
{
    for (unsigned int i=0; i<axes.size(); ++i) {
        addAxesToRemove(axes[i]);
    }
}

void biorbd::rigidbody::NodeSegment::addAxesToRemove(const
        std::vector<biorbd::utils::String>& axes)
{
    for (unsigned int i=0; i<axes.size(); ++i) {
        addAxesToRemove(axes[i]);
    }
}

biorbd::utils::String biorbd::rigidbody::NodeSegment::axesToRemove()
{
    biorbd::utils::String axes;
    if (isAxisRemoved(0)) {
        axes += "x";
    }
    if (isAxisRemoved(1)) {
        axes += "y";
    }
    if (isAxisRemoved(2)) {
        axes += "z";
    }
    return axes;
}
