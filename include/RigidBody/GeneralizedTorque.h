#ifndef BIORBD_UTILS_GENERALIZED_TORQUE_H
#define BIORBD_UTILS_GENERALIZED_TORQUE_H

#include "biorbdConfig.h"
#include "Utils/Vector.h"

namespace biorbd
{
namespace rigidbody
{
class Joints;

///
/// \brief Class GeneralizedTorque
///
class BIORBD_API GeneralizedTorque : public biorbd::utils::Vector
{
public:

    ///
    /// \brief Construct generalized torque
    ///
    GeneralizedTorque();

    ///
    /// \brief Construct generalized torque of dimension nTorque
    /// \param nTorque Position of the vector
    ///
    GeneralizedTorque(
        unsigned int nTorque);

    ///
    /// \brief Construct generalized torque from a joint model
    /// \param j The joint model
    ///
    GeneralizedTorque(const biorbd::rigidbody::Joints& j);

    ///
    /// \brief Construct generalized torque from anoter Generalized torque
    /// \param other The other generalized torque
    ///
    GeneralizedTorque(
        const biorbd::rigidbody::GeneralizedTorque& other);

    ///
    /// \brief Construct vector from Casadi vector
    /// \param v The vector to copy
    ///
    GeneralizedTorque(
        const RigidBodyDynamics::Math::VectorNd& v);

#ifdef BIORBD_USE_EIGEN3_MATH

    ///
    /// \brief Construct generalized torque
    /// \param other The other generalized torque
    ///
    template<typename OtherDerived> GeneralizedTorque(
        const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::utils::Vector(other) {}

#endif

#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief Construct vector from Casadi vector
    /// \param v The vector to copy
    ///
    GeneralizedTorque(
        const casadi::MX& v);

#endif


#ifndef SWIG

#ifdef BIORBD_USE_EIGEN3_MATH

    ///
    /// \brief Allows for operator= to be used
    /// \param other
    /// \return The current Generalized Torque
    ///
    template<typename OtherDerived>
    biorbd::rigidbody::GeneralizedTorque& operator=(const Eigen::MatrixBase
            <OtherDerived>& other)
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

#endif // BIORBD_UTILS_GENERALIZED_TORQUE_H
