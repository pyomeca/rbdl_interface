#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedAcceleration.h"

#include "RigidBody/Joints.h"

biorbd::rigidbody::GeneralizedAcceleration::GeneralizedAcceleration()
{

}

biorbd::rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
    unsigned int nQddot) :
    biorbd::utils::Vector(nQddot)
{

}

biorbd::rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
    const biorbd::rigidbody::Joints &j) :
    biorbd::utils::Vector(j.nbQ())
{

}

biorbd::rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
    const biorbd::rigidbody::GeneralizedAcceleration &Q) :
    biorbd::utils::Vector(Q)
{

}

biorbd::rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
    const RigidBodyDynamics::Math::VectorNd &v) :
    biorbd::utils::Vector (v)
{

}

#ifdef BIORBD_USE_CASADI_MATH

biorbd::rigidbody::GeneralizedAcceleration::GeneralizedAcceleration(
    const casadi::MX &v) :
    biorbd::utils::Vector(v)
{

}

#endif


biorbd::rigidbody::GeneralizedAcceleration::~GeneralizedAcceleration()
{

}

void biorbd::rigidbody::GeneralizedAcceleration::operator=(
    const biorbd::utils::Vector &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void biorbd::rigidbody::GeneralizedAcceleration::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

void biorbd::rigidbody::GeneralizedAcceleration::operator=(
    const casadi::MX &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#endif
