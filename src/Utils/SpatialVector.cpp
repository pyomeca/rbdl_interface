#define BIORBD_API_EXPORTS
#include "Utils/SpatialVector.h"

#include "Utils/Error.h"
#include "Utils/String.h"
#include "Utils/Vector3d.h"

biorbd::utils::SpatialVector::SpatialVector() :
    RigidBodyDynamics::Math::SpatialVector()
{

}

biorbd::utils::SpatialVector::SpatialVector(
    const biorbd::utils::SpatialVector& other) :
    RigidBodyDynamics::Math::SpatialVector (other)
{

}

biorbd::utils::SpatialVector::SpatialVector(
    Scalar v1, Scalar v2, Scalar v3,
    Scalar v4, Scalar v5, Scalar v6) :
    RigidBodyDynamics::Math::SpatialVector (v1, v2, v3, v4, v5, v6)
{

}

#ifdef BIORBD_USE_CASADI_MATH

biorbd::utils::SpatialVector::SpatialVector(
    const casadi::MX &v) :
    RigidBodyDynamics::Math::SpatialVector(v)
{

}

biorbd::utils::SpatialVector::SpatialVector(
    const RBDLCasadiMath::MX_Xd_SubMatrix &m) :
    RigidBodyDynamics::Math::SpatialVector (m)
{

}

#endif

void biorbd::utils::SpatialVector::operator=(
    const biorbd::utils::SpatialVector &other)
{
    this->RigidBodyDynamics::Math::SpatialVector::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void biorbd::utils::SpatialVector::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix& other)
{
    this->RigidBodyDynamics::Math::SpatialVector::operator=(other);
}

void biorbd::utils::SpatialVector::operator=(
    const casadi::MX &other)
{
    this->RigidBodyDynamics::Math::SpatialVector::operator=(other);
}

#endif
