#ifndef BIORBD_RIGIDBODY_SEGMENT_CHARACTERISTICS_H
#define BIORBD_RIGIDBODY_SEGMENT_CHARACTERISTICS_H

#include <memory>
#include <rbdl/Body.h>
#include "biorbdConfig.h"

#include "Utils/Scalar.h"

namespace biorbd
{
namespace utils
{
class Vector3d;
}

namespace rigidbody
{
class Mesh;

///
/// \brief Characteristics of a segment, namely the mass, the center of mass,
/// the inertia, the length and its mesh geometry
///
#ifdef SWIG
class BIORBD_API SegmentCharacteristics
#else
class BIORBD_API SegmentCharacteristics : public RigidBodyDynamics::Body
#endif
{
public:
    ///
    /// \brief Construct segment characteristics
    ///
    SegmentCharacteristics();

    ///
    /// \brief Construct segment characteristics
    /// \param mass The mass of the segment
    /// \param com The position of the center of Mass
    /// \param inertia The inertia matrix
    ///
    SegmentCharacteristics(
        const biorbd::utils::Scalar& mass,
        const biorbd::utils::Vector3d &com,
        const RigidBodyDynamics::Math::Matrix3d &inertia);

    ///
    /// \brief Construct segment characteristics
    /// \param mass The mass of the segment
    /// \param com The position of the center of Mass
    /// \param inertia The inertia matrix
    /// \param mesh The mesh geometry of the segment
    ///
    SegmentCharacteristics(
        const biorbd::utils::Scalar &mass,
        const biorbd::utils::Vector3d &com,
        const RigidBodyDynamics::Math::Matrix3d &inertia,
        const biorbd::rigidbody::Mesh &mesh);

    ///
    /// \brief Deep copy of the segment characteristics
    /// \return Copy of the segment characteristics
    ///
    biorbd::rigidbody::SegmentCharacteristics DeepCopy() const;

    ///
    /// \brief Copy the segment characteristics
    /// \param other The characteristics to copy
    ///
    void DeepCopy(const biorbd::rigidbody::SegmentCharacteristics& other);

    ///
    /// \brief Set the segment length
    /// \param val Value of the new length
    ///
    void setLength(
        const biorbd::utils::Scalar& val);

    ///
    /// \brief Returns the segment length
    /// \return The segment length
    ///
    biorbd::utils::Scalar length() const;

    ///
    /// \brief Set the new mass
    /// \param newMass The new mass
    ///
    void setMass(const biorbd::utils::Scalar& newMass);

    ///
    /// \brief Returns the segment mass
    /// \return The segment mass
    ///
    biorbd::utils::Scalar mass() const;

    ///
    /// \brief CoM Returns the position of the center of mass in the local reference frame
    /// \return The position of the center of mass in the local reference frame
    ///
    biorbd::utils::Vector3d CoM() const;

    ///
    /// \brief setCoM Change the position of the center of mass
    /// \param com The new position for the CoM
    ///
    void setCoM(const biorbd::utils::Vector3d& com);

    ///
    /// \brief Returns the segment mesh
    /// \return The segment mesh
    ///
    const biorbd::rigidbody::Mesh& mesh() const;

    ///
    /// \brief Returns the segment inertia matrix
    /// \return The segment inertia matrix
    ///
    const RigidBodyDynamics::Math::Matrix3d& inertia() const;

protected:
    std::shared_ptr<biorbd::utils::Scalar> m_length; ///< Length of the segment
    std::shared_ptr<biorbd::rigidbody::Mesh> m_mesh; ///< Mesh of the segment
};

}
}

#endif // BIORBD_RIGIDBODY_SEGMENT_CHARACTERISTICS_H
