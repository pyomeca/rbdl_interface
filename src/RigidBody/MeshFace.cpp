#define BIORBD_API_EXPORTS
#include "RigidBody/MeshFace.h"

#include "Utils/Vector3d.h"

biorbd::rigidbody::MeshFace::MeshFace(const std::vector<int>& vertex) :
    m_face(std::make_shared<std::vector<int>>(vertex))
{

}

biorbd::rigidbody::MeshFace biorbd::rigidbody::MeshFace::DeepCopy() const
{
    biorbd::rigidbody::MeshFace copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::MeshFace::DeepCopy(const biorbd::rigidbody::MeshFace
        &other)
{
    *m_face = *other.m_face;
}

int &biorbd::rigidbody::MeshFace::operator()(unsigned int idx)
{
    return (*m_face)[idx];
}

biorbd::utils::Vector3d biorbd::rigidbody::MeshFace::faceAsDouble()
{
    return biorbd::utils::Vector3d(static_cast<double>((*m_face)[0]),
                                   static_cast<double>((*m_face)[1]),
                                   static_cast<double>((*m_face)[2]));
}

std::vector<int> biorbd::rigidbody::MeshFace::face()
{
    return *m_face;
}

void biorbd::rigidbody::MeshFace::setFace(const std::vector<int> & pts)
{
    *m_face = pts;
}

void biorbd::rigidbody::MeshFace::setFace(const biorbd::rigidbody::MeshFace
        &other)
{
    m_face = other.m_face;
}

