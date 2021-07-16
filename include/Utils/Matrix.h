#ifndef BIORBD_UTILS_MATRIX_H
#define BIORBD_UTILS_MATRIX_H

#include "biorbdConfig.h"
#include "rbdl/rbdl_math.h"

namespace biorbd
{
namespace rigidbody
{
class GeneralizedCoordinates;
}

namespace utils
{
///
/// \brief A wrapper for the Eigen::MatrixXd
///
#ifdef SWIG
class BIORBD_API Matrix
#else
class BIORBD_API Matrix : public RigidBodyDynamics::Math::MatrixNd
#endif
{
public:
    ///
    /// \brief Construct matrix
    ///
    Matrix();

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct matrix from another Eigen matrix
    /// \param other The other Eigen matrix
    ///
    template<typename OtherDerived> Matrix(const Eigen::MatrixBase<OtherDerived>&
                                           other) :
        Eigen::MatrixXd(other) {}
#endif
#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief Construct matrix from Casadi matrix
    /// \param other The matrix to copy
    ///
    Matrix(
        const biorbd::utils::Matrix& other);

    ///
    /// \brief Construct matrix from Casadi matrix
    /// \param other The matrix to copy
    ///
    Matrix(
        const RigidBodyDynamics::Math::MatrixNd& other);

    ///
    /// \brief Construct matrix from Casadi matrix
    /// \param other The matrix to copy
    ///
    Matrix(
        const RBDLCasadiMath::MX_Xd_SubMatrix& other);

#endif

    ///
    /// \brief Construct matrix of size nbRows,nbCols
    /// \param nbRows Number of rows
    /// \param nbCols Number of columns
    ///
    Matrix(
        unsigned int nbRows,
        unsigned int nbCols);

#ifndef SWIG

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief To use operator= with matrix
    /// \param other The other Eigen matrix
    ///
    template<typename OtherDerived>
    biorbd::utils::Matrix& operator=(const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->Eigen::MatrixXd::operator=(other);
        return *this;
    }
#endif
#ifdef BIORBD_USE_CASADI_MATH

    ///
    /// \brief operator= For submatrices
    /// \param other The matrix to copy
    ///
    void operator=(
        const biorbd::utils::Matrix& other);

    ///
    /// \brief operator= For submatrices
    /// \param other The matrix to copy
    ///
    void operator=(
        const RBDLCasadiMath::MX_Xd_SubMatrix& other);
#endif

#endif
};

}
}

#endif // BIORBD_UTILS_MATRIX_H
