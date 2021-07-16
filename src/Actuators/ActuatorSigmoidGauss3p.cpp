#define BIORBD_API_EXPORTS
#include "Actuators/ActuatorSigmoidGauss3p.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"

biorbd::actuator::ActuatorSigmoidGauss3p::ActuatorSigmoidGauss3p() :
    biorbd::actuator::Actuator(),
    m_theta(std::make_shared<biorbd::utils::Scalar>(0)),
    m_lambda(std::make_shared<biorbd::utils::Scalar>(0)),
    m_offset(std::make_shared<biorbd::utils::Scalar>(0)),
    m_r(std::make_shared<biorbd::utils::Scalar>(0)),
    m_qopt(std::make_shared<biorbd::utils::Scalar>(0))
{
    setType();
}

biorbd::actuator::ActuatorSigmoidGauss3p::ActuatorSigmoidGauss3p(
    const biorbd::actuator::ActuatorSigmoidGauss3p &other) :
    biorbd::actuator::Actuator(other),
    m_theta(other.m_theta),
    m_lambda(other.m_lambda),
    m_offset(other.m_offset),
    m_r(other.m_r),
    m_qopt(other.m_qopt)
{

}

biorbd::actuator::ActuatorSigmoidGauss3p::ActuatorSigmoidGauss3p(
    int direction,
    const biorbd::utils::Scalar& theta,
    const biorbd::utils::Scalar& lambda,
    const biorbd::utils::Scalar& offset,
    const biorbd::utils::Scalar& r,
    const biorbd::utils::Scalar& qopt,
    unsigned int dofIdx) :
    biorbd::actuator::Actuator(direction, dofIdx),
    m_theta(std::make_shared<biorbd::utils::Scalar>(theta)),
    m_lambda(std::make_shared<biorbd::utils::Scalar>(lambda)),
    m_offset(std::make_shared<biorbd::utils::Scalar>(offset)),
    m_r(std::make_shared<biorbd::utils::Scalar>(r)),
    m_qopt(std::make_shared<biorbd::utils::Scalar>(qopt))
{
    setType();
}

biorbd::actuator::ActuatorSigmoidGauss3p::ActuatorSigmoidGauss3p(
    int direction,
    const biorbd::utils::Scalar& theta,
    const biorbd::utils::Scalar& lambda,
    const biorbd::utils::Scalar& offset,
    const biorbd::utils::Scalar& r,
    const biorbd::utils::Scalar& qopt,
    unsigned int dofIdx,
    const biorbd::utils::String &jointName) :
    biorbd::actuator::Actuator(direction, dofIdx, jointName),
    m_theta(std::make_shared<biorbd::utils::Scalar>(theta)),
    m_lambda(std::make_shared<biorbd::utils::Scalar>(lambda)),
    m_offset(std::make_shared<biorbd::utils::Scalar>(offset)),
    m_r(std::make_shared<biorbd::utils::Scalar>(r)),
    m_qopt(std::make_shared<biorbd::utils::Scalar>(qopt))
{
    setType();
}

biorbd::actuator::ActuatorSigmoidGauss3p::~ActuatorSigmoidGauss3p()
{

}

biorbd::actuator::ActuatorSigmoidGauss3p
biorbd::actuator::ActuatorSigmoidGauss3p::DeepCopy() const
{
    biorbd::actuator::ActuatorSigmoidGauss3p copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::actuator::ActuatorSigmoidGauss3p::DeepCopy(
    const biorbd::actuator::ActuatorSigmoidGauss3p &other)
{
    biorbd::actuator::Actuator::DeepCopy(other);
    *m_theta = *other.m_theta;
    *m_lambda = *other.m_lambda;
    *m_offset = *other.m_offset;
    *m_r = *other.m_r;
    *m_qopt = *other.m_qopt;
}

biorbd::utils::Scalar biorbd::actuator::ActuatorSigmoidGauss3p::torqueMax()
{
    biorbd::utils::Error::raise(
        "torqueMax for ActuatorSigmoidGauss3p must be called with Q and Qdot");
}

biorbd::utils::Scalar biorbd::actuator::ActuatorSigmoidGauss3p::torqueMax(
    const biorbd::rigidbody::GeneralizedCoordinates &Q,
    const biorbd::rigidbody::GeneralizedVelocity &Qdot)
{
    biorbd::utils::Scalar pos(Q[*m_dofIdx] * 180/M_PI);
    biorbd::utils::Scalar speed(Qdot[*m_dofIdx] * 180/M_PI);

    // Getting Tmax of Gauss3p from Sigmoid
    biorbd::utils::Scalar Tmax(*m_theta / (1 + exp(*m_lambda * speed)) + *m_offset);

    // Calculation of the max torque
    return Tmax * exp(-(*m_qopt - pos) * (*m_qopt - pos) / (2 * *m_r * *m_r));
}

void biorbd::actuator::ActuatorSigmoidGauss3p::setType()
{
    *m_type = biorbd::actuator::TYPE::SIGMOIDGAUSS3P;
}
