#ifndef BIORBD_UTILS_VECTOR3D_H
#define BIORBD_UTILS_VECTOR3D_H

#include <memory>
#include "biorbdConfig.h"
#include "rbdl/rbdl_math.h"
#include "Utils/Node.h"
#include "Utils/Scalar.h"

namespace biorbd
{
namespace utils
{
class RotoTrans;
class String;
class Vector;

///
/// \brief Wrapper around Eigen Vector3d and attach it to a parent
///
#ifdef SWIG
class BIORBD_API Vector3d
#else
class BIORBD_API Vector3d : public RigidBodyDynamics::Math::Vector3d,
    public biorbd::utils::Node
#endif
{
public:
    ///
    /// \brief Construct 3D vector
    ///
    Vector3d();

    ///
    /// \brief Construct 3D vector
    /// \param x X-Component of the vector
    /// \param y Y-Component of the vector
    /// \param z Z-Component of the vector
    ///
    Vector3d(
        const biorbd::utils::Scalar& x,
        const biorbd::utils::Scalar& y,
        const biorbd::utils::Scalar& z);

    ///
    /// \brief Construct a 3D vector
    /// \param x X-Component of the vector
    /// \param y Y-Component of the vector
    /// \param z Z-Component of the vector
    /// \param name Name of the vector
    /// \param parentName Name of the parent segment
    ///
    Vector3d(
        const biorbd::utils::Scalar& x,
        const biorbd::utils::Scalar& y,
        const biorbd::utils::Scalar& z,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName);

    ///
    /// \brief Construct a 3D vector
    /// \param vec The vector to copy
    /// \param name Name of the vector
    /// \param parentName Name of the parent segment
    ///
    Vector3d(
        const biorbd::utils::Vector3d vec,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName);

    ///
    /// \brief Construct a 3D vector from a Casadi 3D vector (drop the trailling 1)
    /// \param other The Casadi 3D vector
    ///
    Vector3d(
        const RigidBodyDynamics::Math::Vector3d& other);

    ///
    /// \brief Construct a 3D vector from a Casadi ND vector (drop the trailling 1)
    /// \param other The Casadi ND vector
    ///
    Vector3d(
        const RigidBodyDynamics::Math::VectorNd& other);

    ///
    /// \brief Construct a 3D vector from an eigen 4D vector (drop the trailling 1)
    /// \param other The Eigen 4D vector
    ///
    Vector3d(
        const RigidBodyDynamics::Math::Vector4d& other);

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct a 3D vector
    /// \param other The other vector
    ///
    template<typename OtherDerived> Vector3d(
        const Eigen::MatrixBase<OtherDerived>& other) :
        RigidBodyDynamics::Math::Vector3d(other), biorbd::utils::Node ()
    {
    }

    ///
    /// \brief Construct a 3D vector
    /// \param other Position of the vector (eigen matrix)
    /// \param name Name of the vector
    /// \param parentName The name of the parent segment
    ///
    template<typename OtherDerived> Vector3d(
        const Eigen::MatrixBase<OtherDerived>& other,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName) :
        RigidBodyDynamics::Math::Vector3d(other), biorbd::utils::Node (name, parentName)
    {

    }
#endif
#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief Construct a 3D vector from a Casadi ND vector (drop the trailling 1)
    /// \param other The Casadi ND vector
    ///
    Vector3d(
        const RBDLCasadiMath::MX_Xd_SubMatrix& other);
#endif

    ///
    /// \brief Deep copy of a 3D vector
    /// \return A deep copy of a 3D vector
    ///
    biorbd::utils::Vector3d DeepCopy() const;

    ///
    /// \brief Deep copy of a 3D vector into another 3D vector
    /// \param other The 3D vector to copy
    ///
    void DeepCopy(const biorbd::utils::Vector3d& other);

    ///
    /// \brief Apply a RotoTrans to the 3D vector
    /// \param rt RotoTrans to apply
    /// \return The transformed vector
    //
    biorbd::utils::Vector3d applyRT(
        const RotoTrans& rt) const;

    ///
    /// \brief Apply a RotoTrans to the 3D vector
    /// \param rt RotoTrans to apply
    ///
    void applyRT(
        const RotoTrans& rt);

    ///
    /// \brief Set a new position
    /// \param v The new position
    ///
    void setPosition(
        const biorbd::utils::Vector3d& v);

#ifndef SWIG

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief To use operator= on 3D vector with any eigen vector
    /// \param other The eigen matrix
    ///
    template<typename OtherDerived>
    biorbd::utils::Vector3d& operator=(const Eigen::MatrixBase <OtherDerived>&
                                       other)
    {
        this->Eigen::Vector3d::operator=(other);
        return *this;
    }
#endif

#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief Accessor for the first element
    /// \return The first element
    ///
    biorbd::utils::Scalar x() const;

    ///
    /// \brief Accessor for the second element
    /// \return The second element
    ///
    biorbd::utils::Scalar y() const;

    ///
    /// \brief Accessor for the third element
    /// \return The third element
    ///
    biorbd::utils::Scalar z() const;

    ///
    /// \brief Construct a 3D vector from a Casadi 4D vector (drop the trailling 1)
    /// \param other The Casadi 4D vector
    ///
    template<unsigned int i, unsigned int j>
    void operator=(
        const MX_Xd_static<i, j>& other)
    {
        this->block<3, 1>(0, 0) = other;
    }

    ///
    /// \brief operator= To copy a submatrix
    /// \param other The matrix to copy
    ///
    void operator=(
        const RBDLCasadiMath::MX_Xd_SubMatrix& other);

    ///
    /// \brief operator= To copy a vector
    /// \param other The vector to copy
    ///
    void operator=(
        const RigidBodyDynamics::Math::Vector4d& other);

#endif

#endif

protected:
    ///
    /// \brief Set the type Vector3d
    ///
    void setType();
};

}
}

#endif // BIORBD_UTILS_VECTOR3D_H
