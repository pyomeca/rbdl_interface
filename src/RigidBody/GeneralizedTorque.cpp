#define BIORBD_API_EXPORTS
#include "RigidBody/GeneralizedTorque.h"

#include "RigidBody/Joints.h"

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque()
{

}

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque(
    unsigned int nTorque) :
    biorbd::utils::Vector(nTorque)
{

}

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque(
    const biorbd::rigidbody::Joints &j) :
    biorbd::utils::Vector(j.nbGeneralizedTorque())
{

}

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque(
    const biorbd::rigidbody::GeneralizedTorque &other) :
    biorbd::utils::Vector(other)
{

}

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque(
    const RigidBodyDynamics::Math::VectorNd &v) :
    biorbd::utils::Vector (v)
{

}

#ifdef BIORBD_USE_CASADI_MATH

biorbd::rigidbody::GeneralizedTorque::GeneralizedTorque(
    const casadi::MX &v) :
    biorbd::utils::Vector(v)
{

}

#endif

void biorbd::rigidbody::GeneralizedTorque::operator=(
    const biorbd::utils::Vector &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#ifdef BIORBD_USE_CASADI_MATH

void biorbd::rigidbody::GeneralizedTorque::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

void biorbd::rigidbody::GeneralizedTorque::operator=(
    const casadi::MX &other)
{
    this->biorbd::utils::Vector::operator=(other);
}

#endif
