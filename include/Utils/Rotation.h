#ifndef BIORBD_UTILS_ROTATION_H
#define BIORBD_UTILS_ROTATION_H

#include <vector>
#include <memory>
#include "rbdl/rbdl_math.h"
#include "Utils/Scalar.h"

namespace RigidBodyDynamics
{
namespace Math
{
struct SpatialTransform;
}
}

#include "biorbdConfig.h"

namespace biorbd
{
namespace rigidbody
{
class NodeSegment;
}

namespace utils
{
class String;
class Vector;
class Vector3d;
class Matrix;

///
/// \brief Rotation matrix
///
#ifdef SWIG
class BIORBD_API Rotation
#else
class BIORBD_API Rotation : public RigidBodyDynamics::Math::Matrix3d
#endif
{
public:
    ///
    /// \brief Construct Rotation matrix
    /// \param matrix 3D identity matrix
    ///
    Rotation(
        const RigidBodyDynamics::Math::Matrix3d& matrix =
            RigidBodyDynamics::Math::Matrix3d::Identity());

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct a Rotation matrix from another Rotation
    /// \param other The other Rotation
    ///
    template<typename OtherDerived> Rotation(
        const Eigen::MatrixBase<OtherDerived>& other) :
        RigidBodyDynamics::Math::Matrix3d(other)
    {
        checkUnitary();
    }
#endif
#ifdef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Construct Rotation matrix
    /// \param matrix 3D identity matrix
    ///
    Rotation(
        const RigidBodyDynamics::Math::MatrixNd& m);
#endif

    ///
    /// \brief Rotation Construct a Rotation matrix by elements
    /// \param v00 Row 0, Col 0
    /// \param v01 Row 0, Col 1
    /// \param v02 Row 0, Col 2
    /// \param v10 Row 1, Col 0
    /// \param v11 Row 1, Col 1
    /// \param v12 Row 1, Col 2
    /// \param v20 Row 2, Col 0
    /// \param v21 Row 2, Col 1
    /// \param v22 Row 2, Col 2
    ///
    Rotation(const biorbd::utils::Scalar& v00, const biorbd::utils::Scalar& v01,
             const biorbd::utils::Scalar& v02,
             const biorbd::utils::Scalar& v10, const biorbd::utils::Scalar& v11,
             const biorbd::utils::Scalar& v12,
             const biorbd::utils::Scalar& v20, const biorbd::utils::Scalar& v21,
             const biorbd::utils::Scalar& v22);

    ///
    /// \brief Contruct a Rotation matrix
    /// \param rotation The Euler angles vector
    /// \param rotationSequence The rotation sequence
    ///
    /// The number of rotation must match the number of axes in the rotation
    /// sequence
    ///
    Rotation(
        const biorbd::utils::Vector& rotation,
        const biorbd::utils::String& rotationSequence);

    ///
    /// \brief Contruct Rototrans
    /// \param st Spatial Transform vector
    ///
    Rotation(
        const RigidBodyDynamics::Math::SpatialTransform& st);

    ///
    /// \brief Get a particular axis of the Rotation matrix
    /// \param idx The index of axis (x = 0, y = 1 and z = 2)
    /// \return The axis
    ///
    biorbd::utils::Vector3d axe(
        unsigned int idx) const;

    ///
    /// \brief set the Rotation from a spatial transform
    /// \param st The spatial transform
    /// \return The matrix of Rotation
    ///
    static biorbd::utils::Rotation fromSpatialTransform(
        const RigidBodyDynamics::Math::SpatialTransform& st);

    ///
    /// \brief Create a Rotation from Euler angles
    /// \param rot The Euler angles vector
    /// \param seq The rotation sequence
    ///
    /// The number of rotation must match the number of axes in the rotation
    /// sequence
    ///
    static biorbd::utils::Rotation fromEulerAngles(
        const biorbd::utils::Vector& rot,
        const biorbd::utils::String& seq);

    ///
    /// \brief fromMarkers Creates a system of axes from two axes defined by markers
    /// \param axis1markers The beginning and ending of the vector of the first axis
    /// \param axis2markers The beginning and ending of the vector of the second axis
    /// \param axesNames The names ("x", "y" or "z") of the axes
    /// \param axisToRecalculate The axis to recalculate to ensure orthonormal system of axes
    /// \return The system of axes
    ///
    static biorbd::utils::Matrix fromMarkersNonNormalized(
        const std::pair<biorbd::rigidbody::NodeSegment, biorbd::rigidbody::NodeSegment>&
        axis1markers,
        const std::pair<biorbd::rigidbody::NodeSegment, biorbd::rigidbody::NodeSegment>&
        axis2markers,
        const std::pair<biorbd::utils::String, biorbd::utils::String> &axesNames,
        const biorbd::utils::String& axisToRecalculate);

    ///
    /// \brief fromMarkers Creates a system of axes from two axes defined by markers
    /// \param axis1markers The beginning and ending of the vector of the first axis
    /// \param axis2markers The beginning and ending of the vector of the second axis
    /// \param axesNames The names ("x", "y" or "z") of the axes
    /// \param axisToRecalculate The axis to recalculate to ensure orthonormal system of axes
    /// \return The system of axes
    ///
    static biorbd::utils::Rotation fromMarkers(
        const std::pair<biorbd::rigidbody::NodeSegment, biorbd::rigidbody::NodeSegment>&
        axis1markers,
        const std::pair<biorbd::rigidbody::NodeSegment, biorbd::rigidbody::NodeSegment>&
        axis2markers,
        const std::pair<biorbd::utils::String, biorbd::utils::String> &axesNames,
        const biorbd::utils::String& axisToRecalculate);

    ///
    /// \brief Return extracted angles from the rotation matrix into Euler angles using the provided sequence
    /// \param r The Rotation matrix to extract angles from
    /// \param seq The angle sequence
    /// \return The angles (the length of the vector will match the length of sequence)
    ///
    /// The rotation sequence can be any combination of x, y and z
    ///
    static  biorbd::utils::Vector toEulerAngles(
        const biorbd::utils::Rotation& r,
        const biorbd::utils::String& seq);

#ifndef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Get the mean of the Rotation matrices
    /// \param mToMean The Rotation matrices to mean
    /// \return The mean Rotation matrix
    ///
    static biorbd::utils::Rotation mean(
        const std::vector<biorbd::utils::Rotation>& mToMean);
#endif

#ifndef SWIG
#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Allow the use of operator=
    /// \param other The other Rotation matrix
    ///
    template<typename OtherDerived>
    biorbd::utils::Rotation& operator=(
        const Eigen::MatrixBase <OtherDerived>& other)
    {
        Eigen::Matrix3d::operator=(other);
        return *this;
    }
#endif
#endif

protected:
    ///
    /// \brief Check if the Rotation is a unitary matrix of rotation
    ///
    /// That function throws a runtime_error if the check fails
    ///
    void checkUnitary();
};

}
}

#ifndef SWIG
    ///
    /// \brief To use operator<< to use std::cout
    /// \param os osstream
    /// \param rt The Rotation matrix
    ///
    std::ostream& operator<<(std::ostream& os, const biorbd::utils::Rotation &rt);
#endif

#endif // BIORBD_UTILS_ROTO_TRANS_H
