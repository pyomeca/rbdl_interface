#define BIORBD_API_EXPORTS
#include "Utils/Matrix.h"

#include "Utils/Vector.h"

biorbd::utils::Matrix::Matrix() :
    RigidBodyDynamics::Math::MatrixNd()
{

}


#ifdef BIORBD_USE_CASADI_MATH

biorbd::utils::Matrix::Matrix(
    const biorbd::utils::Matrix& v) :
    RigidBodyDynamics::Math::MatrixNd (v)
{

}

biorbd::utils::Matrix::Matrix(
    const RigidBodyDynamics::Math::MatrixNd &v) :
    RigidBodyDynamics::Math::MatrixNd (v)
{

}

biorbd::utils::Matrix::Matrix(
    const RBDLCasadiMath::MX_Xd_SubMatrix &m) :
    RigidBodyDynamics::Math::VectorNd (m)
{

}

void biorbd::utils::Matrix::operator=(
    const biorbd::utils::Matrix &other)
{
    this->MX_Xd_dynamic::operator=(other);
}

void biorbd::utils::Matrix::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix& other)
{
    this->MX_Xd_dynamic::operator=(other);
}

#endif


biorbd::utils::Matrix::Matrix(
    unsigned int nbRows,
    unsigned int nbCols) :
    RigidBodyDynamics::Math::MatrixNd(nbRows, nbCols)
{

}
