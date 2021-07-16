#ifndef BIORBD_RIGIDBODY_MESH_H
#define BIORBD_RIGIDBODY_MESH_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"

namespace biorbd
{
namespace utils
{
class Vector3d;
class Path;
}

namespace rigidbody
{
class MeshFace;

///
/// \brief A class that holds the geometry of a segment
///
class BIORBD_API Mesh
{
public:
    ///
    /// \brief Construct mesh
    ///
    Mesh();

    ///
    /// \brief Construct mesh from another mesh
    /// \brief other The other mesh
    ///
    Mesh(
        const std::vector<biorbd::utils::Vector3d>& other);

    ///
    /// \brief Construct mesh
    /// \param vertex The vertex of the geometry
    /// \param faces The faces of the geometry
    ///
    Mesh(
        const std::vector<biorbd::utils::Vector3d>& vertex,
        const std::vector<biorbd::rigidbody::MeshFace>& faces);

    ///
    /// \brief Deep copy of the mesh
    /// \return A copy of mesh
    ///
    biorbd::rigidbody::Mesh DeepCopy() const;

    ///
    /// \brief Deep copy of the mesh
    /// \param other The mesh to copy
    ///
    void DeepCopy(
        const biorbd::rigidbody::Mesh& other);

    ///
    /// \brief Add a point to the mesh
    /// \param node The point to add
    ///
    void addPoint(
        const biorbd::utils::Vector3d& node);

    ///
    /// \brief Return the point of a specific index
    /// \param idx The index of the point
    /// \return The point of a specific index
    ///
    const biorbd::utils::Vector3d& point(
        unsigned int idx) const;

    ///
    /// \brief Returns the number of vertex
    /// \return The number of vertex
    ///
    unsigned int nbVertex() const;

    ///
    /// \brief Add a face patch to the mesh
    /// \param face The face patch to add
    ///
    void addFace(
        const biorbd::rigidbody::MeshFace& face);

    ///
    /// \brief Add a face patch to the mesh
    /// \param face The face patch to add
    ///
    void addFace(const std::vector<int> &face);

    ///
    /// \brief Return the faces of the mesh
    /// \return The faces of the mesh
    ///
    const std::vector<biorbd::rigidbody::MeshFace>& faces() const;

    ///
    /// \brief Return the face of the mesh of a specified idx
    /// \param idx Position
    /// \return The face of the mesh of a specified idx
    ///
    const biorbd::rigidbody::MeshFace& face(
        unsigned int idx) const;

    ///
    /// \brief Return the number of faces
    /// \return The number of faces
    ///
    unsigned int nbFaces();

    ///
    /// \brief Set the path of the underlying mesh file
    /// \param path Path for the mesh file
    ///
    void setPath(
        const biorbd::utils::Path& path);

    ///
    /// \brief Return the path of the mesh file
    /// \return The path of the mesh file
    ///
    const biorbd::utils::Path& path() const;

protected:
    std::shared_ptr<std::vector<biorbd::utils::Vector3d>> m_vertex; ///< The vertex
    std::shared_ptr<std::vector<biorbd::rigidbody::MeshFace>>
            m_faces; ///< The faces
    std::shared_ptr<biorbd::utils::Path> m_pathFile; ///< The path to the mesh file
};

}
}

#endif // BIORBD_RIGIDBODY_MESH_H
