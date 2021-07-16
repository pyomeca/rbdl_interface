#ifndef BIORBD_UTILS_GENERALIZED_VELOCITY_H
#define BIORBD_UTILS_GENERALIZED_VELOCITY_H

#include "biorbdConfig.h"
#include "Utils/Vector.h"

namespace biorbd
{
namespace rigidbody
{
class Joints;

///
/// \brief Class GeneralizedVelocity
///
class BIORBD_API GeneralizedVelocity : public biorbd::utils::Vector
{
public:

    ///
    /// \brief Construct generalized velocity vector
    ///
    GeneralizedVelocity();

    ///
    /// \brief Create generalized velocity vector of dimension nbQdot
    /// \param nbQdot number of degrees-of-freedom
    ///
    GeneralizedVelocity(
        unsigned int nbQdot);

    ///
    /// \brief Create generalized velocity vector from a joint Model
    /// \param j The joint model
    ///
    GeneralizedVelocity(const biorbd::rigidbody::Joints& j);

    ///
    /// \brief Construct generalized velocity vector
    /// \param Q State vector of the internal joints
    ///
    GeneralizedVelocity(
        const biorbd::rigidbody::GeneralizedVelocity& Q);

    ///
    /// \brief Construct vector from Casadi vector
    /// \param v The vector to copy
    ///
    GeneralizedVelocity(
        const RigidBodyDynamics::Math::VectorNd& v);

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct generalized velocity vector from another vector
    /// \param other Eigen matrix
    ///
    template<typename OtherDerived> GeneralizedVelocity(
        const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::utils::Vector(other) {}
#endif
#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief Construct vector from Casadi vector
    /// \param v The vector to copy
    ///
    GeneralizedVelocity(
        const casadi::MX& v);

#endif

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~GeneralizedVelocity();

#ifndef SWIG

#ifdef BIORBD_USE_EIGEN3_MATH

    ///
    /// \brief Allows for operator= to be used
    /// \param other
    /// \return The current Generalized velocity vector
    ///
    template<typename OtherDerived>
    biorbd::rigidbody::GeneralizedVelocity& operator=(
        const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->biorbd::utils::Vector::operator=(other);
        return *this;
    }

#endif

    ///
    /// \brief operator= For submatrices
    /// \param other The vector to copy
    ///
    void operator=(
        const biorbd::utils::Vector& other);

#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief operator= For submatrices
    /// \param The vector to copy
    ///
    void operator=(
        const RBDLCasadiMath::MX_Xd_SubMatrix& other);

    ///
    /// \brief operator= For casadi
    /// \param The vector to copy
    ///
    void operator=(
        const casadi::MX& other);
#endif

#endif
};

}
}

#endif // BIORBD_UTILS_GENERALIZED_VELOCITY_H
