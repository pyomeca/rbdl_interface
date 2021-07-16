#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedCoordinates.h"

#include "RigidBody/Joints.h"

biorbd::rigidbody::GeneralizedCoordinates::GeneralizedCoordinates()
{

}

biorbd::rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(
    unsigned int nbQ) :
    biorbd::utils::Vector(nbQ)
{

}

biorbd::rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(
    const biorbd::rigidbody::Joints &j) :
    biorbd::utils::Vector(j.nbQ())
{

}

biorbd::rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(
    const biorbd::rigidbody::GeneralizedCoordinates &Q) :
    biorbd::utils::Vector(Q)
{

}

biorbd::rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(
    const RigidBodyDynamics::Math::VectorNd &v) :
    biorbd::utils::Vector (v)
{

}

#ifdef BIORBD_USE_CASADI_MATH

biorbd::rigidbody::GeneralizedCoordinates::GeneralizedCoordinates(
    const casadi::MX &v) :
    biorbd::utils::Vector(v)
{

}

#endif

biorbd::rigidbody::GeneralizedCoordinates::~GeneralizedCoordinates()
{

}

void biorbd::rigidbody::GeneralizedCoordinates::operator=(
    const biorbd::utils::Vector &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void biorbd::rigidbody::GeneralizedCoordinates::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

void biorbd::rigidbody::GeneralizedCoordinates::operator=(
    const casadi::MX &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#endif
