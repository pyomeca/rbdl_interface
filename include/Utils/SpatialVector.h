#ifndef BIORBD_UTILS_SPATIAL_VECTOR_H
#define BIORBD_UTILS_SPATIAL_VECTOR_H

#include "biorbdConfig.h"
#include "rbdl/rbdl_math.h"
#include "Utils/Scalar.h"

namespace biorbd
{
namespace utils
{

///
/// \brief Wrapper of the Eigen::Matrix<double, 6, 1> or Casadi::MX(6, 1)
///
#ifdef SWIG
class BIORBD_API SpatialVector
#else
class BIORBD_API SpatialVector : public RigidBodyDynamics::Math::SpatialVector
#endif
{
public:
    ///
    /// \brief Construct SpatialVector
    ///
    SpatialVector();

    ///
    /// \brief Construct SpatialVector from Casadi SpatialVector
    /// \param other The SpatialVector to copy
    ///
    SpatialVector(
        const biorbd::utils::SpatialVector& other);

    ///
    /// \brief Construct SpatialVector by its values
    /// \param v1 First element
    /// \param v2 Second element
    /// \param v3 Third element
    /// \param v4 Forth element
    /// \param v5 Fifth element
    /// \param v6 Sixth element
    ///
    SpatialVector(
        biorbd::utils::Scalar v1, biorbd::utils::Scalar v2, biorbd::utils::Scalar v3,
        biorbd::utils::Scalar v4, biorbd::utils::Scalar v5, biorbd::utils::Scalar v6);

#ifdef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Construct SpatialVector from Casadi SpatialVector
    /// \param v The SpatialVector to copy
    ///
    SpatialVector(
        const casadi::MX& v);

    ///
    /// \brief Construct SpatialVector from Casadi matrix
    /// \param other The SpatialVector to copy
    ///
    SpatialVector(
        const RBDLCasadiMath::MX_Xd_SubMatrix& m);
#endif

#ifndef SWIG
    ///
    /// \brief operator= For submatrices
    /// \param other The SpatialVector to copy
    ///
    void operator=(
        const biorbd::utils::SpatialVector& other);
#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Allow the use operator= on SpatialVector
    /// \param other The other matrix
    ///
    template<typename OtherDerived>
    biorbd::utils::SpatialVector& operator=(const Eigen::MatrixBase <OtherDerived>&
                                            other)
    {
        this->Eigen::Matrix<double, 6, 1>::operator=(other);
        return *this;
    }
#endif
#ifdef BIORBD_USE_CASADI_MATH
    ///
    /// \brief operator= For submatrices
    /// \param other The SpatialVector to copy
    ///
    void operator=(
        const RBDLCasadiMath::MX_Xd_SubMatrix& other);

    ///
    /// \brief operator= For submatrices
    /// \param other The SpatialVector to copy
    ///
    void operator=(
        const casadi::MX& other);
#endif

#endif
};

}
}

#endif // BIORBD_UTILS_SPATIAL_VECTOR_H
