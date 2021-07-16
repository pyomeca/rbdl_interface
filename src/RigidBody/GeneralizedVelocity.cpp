#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedVelocity.h"

#include "RigidBody/Joints.h"

biorbd::rigidbody::GeneralizedVelocity::GeneralizedVelocity()
{

}

biorbd::rigidbody::GeneralizedVelocity::GeneralizedVelocity(
    unsigned int nbQdot) :
    biorbd::utils::Vector(nbQdot)
{

}

biorbd::rigidbody::GeneralizedVelocity::GeneralizedVelocity(
    const biorbd::rigidbody::Joints &j) :
    biorbd::utils::Vector(j.nbQdot())
{

}

biorbd::rigidbody::GeneralizedVelocity::GeneralizedVelocity(
    const biorbd::rigidbody::GeneralizedVelocity &Q) :
    biorbd::utils::Vector(Q)
{

}

biorbd::rigidbody::GeneralizedVelocity::GeneralizedVelocity(
    const RigidBodyDynamics::Math::VectorNd &v) :
    biorbd::utils::Vector (v)
{

}

#ifdef BIORBD_USE_CASADI_MATH

biorbd::rigidbody::GeneralizedVelocity::GeneralizedVelocity(
    const casadi::MX &v) :
    biorbd::utils::Vector(v)
{

}

#endif


biorbd::rigidbody::GeneralizedVelocity::~GeneralizedVelocity()
{

}

void biorbd::rigidbody::GeneralizedVelocity::operator=(
    const biorbd::utils::Vector &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void biorbd::rigidbody::GeneralizedVelocity::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

void biorbd::rigidbody::GeneralizedVelocity::operator=(
    const casadi::MX &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#endif
