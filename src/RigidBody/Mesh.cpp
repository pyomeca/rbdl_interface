#define BIORBD_API_EXPORTS
#include "RigidBody/Mesh.h"

#include "Utils/Path.h"
#include "Utils/Vector3d.h"
#include "RigidBody/MeshFace.h"

biorbd::rigidbody::Mesh::Mesh() :
    m_vertex(std::make_shared<std::vector<biorbd::utils::Vector3d>>()),
    m_faces(std::make_shared<std::vector<biorbd::rigidbody::MeshFace>>()),
    m_pathFile(std::make_shared<biorbd::utils::Path>())
{

}

biorbd::rigidbody::Mesh::Mesh(const std::vector<biorbd::utils::Vector3d> &other)
    :
    m_vertex(std::make_shared<std::vector<biorbd::utils::Vector3d>>(other)),
    m_faces(std::make_shared<std::vector<biorbd::rigidbody::MeshFace>>()),
    m_pathFile(std::make_shared<biorbd::utils::Path>())
{

}

biorbd::rigidbody::Mesh::Mesh(const std::vector<biorbd::utils::Vector3d>
                              &vertex,
                              const std::vector<biorbd::rigidbody::MeshFace> & faces) :
    m_vertex(std::make_shared<std::vector<biorbd::utils::Vector3d>>(vertex)),
    m_faces(std::make_shared<std::vector<biorbd::rigidbody::MeshFace>>(faces)),
    m_pathFile(std::make_shared<biorbd::utils::Path>())
{

}

biorbd::rigidbody::Mesh biorbd::rigidbody::Mesh::DeepCopy() const
{
    biorbd::rigidbody::Mesh copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::Mesh::DeepCopy(const biorbd::rigidbody::Mesh &other)
{
    m_vertex->resize(other.m_vertex->size());
    for (unsigned int i=0; i<other.m_vertex->size(); ++i) {
        (*m_vertex)[i] = (*other.m_vertex)[i].DeepCopy();
    }
    m_faces->resize(other.m_faces->size());
    for (unsigned int i=0; i<other.m_faces->size(); ++i) {
        (*m_faces)[i] = (*other.m_faces)[i].DeepCopy();
    }
    *m_pathFile = other.m_pathFile->DeepCopy();
}

void biorbd::rigidbody::Mesh::addPoint(const biorbd::utils::Vector3d &node)
{
    m_vertex->push_back(node);
}
const biorbd::utils::Vector3d &biorbd::rigidbody::Mesh::point(
    unsigned int idx) const
{
    return (*m_vertex)[idx];
}
unsigned int biorbd::rigidbody::Mesh::nbVertex() const
{
    return static_cast<unsigned int>(m_vertex->size());
}

unsigned int biorbd::rigidbody::Mesh::nbFaces()
{
    return static_cast<unsigned int>(m_faces->size());
}
void biorbd::rigidbody::Mesh::addFace(const biorbd::rigidbody::MeshFace& face)
{
    m_faces->push_back(face);
}
void biorbd::rigidbody::Mesh::addFace(const std::vector<int> & face)
{
    addFace(biorbd::rigidbody::MeshFace(face));
}
const std::vector<biorbd::rigidbody::MeshFace>& biorbd::rigidbody::Mesh::faces()
const
{
    return *m_faces;
}
const biorbd::rigidbody::MeshFace &biorbd::rigidbody::Mesh::face(
    unsigned int idx) const
{
    return (*m_faces)[idx];
}

void biorbd::rigidbody::Mesh::setPath(const biorbd::utils::Path& path)
{
    *m_pathFile = path;
}

const biorbd::utils::Path &biorbd::rigidbody::Mesh::path() const
{
    return *m_pathFile;
}
