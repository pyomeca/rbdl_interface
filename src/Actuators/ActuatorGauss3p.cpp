#define BIORBD_API_EXPORTS
#include "Actuators/ActuatorGauss3p.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"

biorbd::actuator::ActuatorGauss3p::ActuatorGauss3p() :
    biorbd::actuator::Actuator(),
    m_k(std::make_shared<biorbd::utils::Scalar>(4.3)),
    m_Tmax(std::make_shared<biorbd::utils::Scalar>(0)),
    m_T0(std::make_shared<biorbd::utils::Scalar>(0)),
    m_wmax(std::make_shared<biorbd::utils::Scalar>(0)),
    m_wc(std::make_shared<biorbd::utils::Scalar>(0)),
    m_amax(std::make_shared<biorbd::utils::Scalar>(1.0)),
    m_amin(std::make_shared<biorbd::utils::Scalar>(0)),
    m_wr(std::make_shared<biorbd::utils::Scalar>(0)),
    m_w1(std::make_shared<biorbd::utils::Scalar>(0)),
    m_r(std::make_shared<biorbd::utils::Scalar>(0)),
    m_qopt(std::make_shared<biorbd::utils::Scalar>(0))
{
    setType();
}

biorbd::actuator::ActuatorGauss3p::ActuatorGauss3p(
    const biorbd::actuator::ActuatorGauss3p &other) :
    biorbd::actuator::Actuator(other),
    m_k(other.m_k),
    m_Tmax(other.m_Tmax),
    m_T0(other.m_T0),
    m_wmax(other.m_wmax),
    m_wc(other.m_wc),
    m_amax(other.m_amax),
    m_amin(other.m_amin),
    m_wr(other.m_wr),
    m_w1(other.m_w1),
    m_r(other.m_r),
    m_qopt(other.m_qopt)
{

}

biorbd::actuator::ActuatorGauss3p::ActuatorGauss3p(
    int direction,
    const biorbd::utils::Scalar& Tmax,
    const biorbd::utils::Scalar& T0,
    const biorbd::utils::Scalar& wmax,
    const biorbd::utils::Scalar& wc,
    const biorbd::utils::Scalar& amin,
    const biorbd::utils::Scalar& wr,
    const biorbd::utils::Scalar& w1,
    const biorbd::utils::Scalar& r,
    const biorbd::utils::Scalar& qopt,
    unsigned int dofIdx) :
    biorbd::actuator::Actuator(direction, dofIdx),
    m_k(std::make_shared<biorbd::utils::Scalar>(4.3)),
    m_Tmax(std::make_shared<biorbd::utils::Scalar>(Tmax)),
    m_T0(std::make_shared<biorbd::utils::Scalar>(T0)),
    m_wmax(std::make_shared<biorbd::utils::Scalar>(wmax)),
    m_wc(std::make_shared<biorbd::utils::Scalar>(wc)),
    m_amax(std::make_shared<biorbd::utils::Scalar>(1.0)),
    m_amin(std::make_shared<biorbd::utils::Scalar>(amin)),
    m_wr(std::make_shared<biorbd::utils::Scalar>(wr)),
    m_w1(std::make_shared<biorbd::utils::Scalar>(w1)),
    m_r(std::make_shared<biorbd::utils::Scalar>(r)),
    m_qopt(std::make_shared<biorbd::utils::Scalar>(qopt))
{
    setType();
}

biorbd::actuator::ActuatorGauss3p::ActuatorGauss3p(
    int direction,
    const biorbd::utils::Scalar& Tmax,
    const biorbd::utils::Scalar& T0,
    const biorbd::utils::Scalar& wmax,
    const biorbd::utils::Scalar& wc,
    const biorbd::utils::Scalar& amin,
    const biorbd::utils::Scalar& wr,
    const biorbd::utils::Scalar& w1,
    const biorbd::utils::Scalar& r,
    const biorbd::utils::Scalar& qopt,
    unsigned int dofIdx,
    const biorbd::utils::String &jointName) :
    biorbd::actuator::Actuator(direction, dofIdx, jointName),
    m_k(std::make_shared<biorbd::utils::Scalar>(4.3)),
    m_Tmax(std::make_shared<biorbd::utils::Scalar>(Tmax)),
    m_T0(std::make_shared<biorbd::utils::Scalar>(T0)),
    m_wmax(std::make_shared<biorbd::utils::Scalar>(wmax)),
    m_wc(std::make_shared<biorbd::utils::Scalar>(wc)),
    m_amax(std::make_shared<biorbd::utils::Scalar>(1.0)),
    m_amin(std::make_shared<biorbd::utils::Scalar>(amin)),
    m_wr(std::make_shared<biorbd::utils::Scalar>(wr)),
    m_w1(std::make_shared<biorbd::utils::Scalar>(w1)),
    m_r(std::make_shared<biorbd::utils::Scalar>(r)),
    m_qopt(std::make_shared<biorbd::utils::Scalar>(qopt))
{
    setType();
}

biorbd::actuator::ActuatorGauss3p::~ActuatorGauss3p()
{

}

biorbd::actuator::ActuatorGauss3p biorbd::actuator::ActuatorGauss3p::DeepCopy()
const
{
    biorbd::actuator::ActuatorGauss3p copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::actuator::ActuatorGauss3p::DeepCopy(
    const biorbd::actuator::ActuatorGauss3p &other)
{
    biorbd::actuator::Actuator::DeepCopy(other);
    *m_k = *other.m_k;
    *m_Tmax = *other.m_Tmax;
    *m_T0 = *other.m_T0;
    *m_wmax = *other.m_wmax;
    *m_wc = *other.m_wc;
    *m_amax = *other.m_amax;
    *m_amin = *other.m_amin;
    *m_wr = *other.m_wr;
    *m_w1 = *other.m_w1;
    *m_r = *other.m_r;
    *m_qopt = *other.m_qopt;
}

biorbd::utils::Scalar biorbd::actuator::ActuatorGauss3p::torqueMax()
{
    biorbd::utils::Error::raise(
        "torqueMax for ActuatorGauss3p must be called with Q and Qdot");
}

biorbd::utils::Scalar biorbd::actuator::ActuatorGauss3p::torqueMax(
    const biorbd::rigidbody::GeneralizedCoordinates &Q,
    const biorbd::rigidbody::GeneralizedVelocity &Qdot)
{
    biorbd::utils::Scalar pos(Q[*m_dofIdx] * 180/M_PI);
    biorbd::utils::Scalar speed(Qdot[*m_dofIdx] * 180/M_PI);

    // Tetanic torque max
    biorbd::utils::Scalar Tc = *m_T0 * *m_wc / *m_wmax;
    biorbd::utils::Scalar C = Tc * (*m_wmax + *m_wc); // concentric
    biorbd::utils::Scalar we =
        ( (*m_Tmax - *m_T0) * *m_wmax * *m_wc )
        / ( *m_k * *m_T0 * (*m_wmax + *m_wc) );
    biorbd::utils::Scalar E = -( *m_Tmax - *m_T0 ) * we; // excentric

    biorbd::utils::Scalar Tw;
#ifdef BIORBD_USE_CASADI_MATH
    Tw = casadi::MX::if_else(casadi::MX::ge(speed, 0),
                             C / ( *m_wc + speed )  - Tc,
                             E / ( we - speed ) + *m_Tmax);
#else
    if (speed >= 0) {
        Tw = C / ( *m_wc + speed )  - Tc;    // For the concentric
    } else {
        Tw = E / ( we - speed ) + *m_Tmax;    // For the excentric
    }
#endif


    // Differential activation
    biorbd::utils::Scalar A =
        *m_amin + ( *m_amax - *m_amin )
        / ( 1 + biorbd::utils::Scalar(exp( -(speed - *m_w1) / *m_wr   )) );

    // Torque angle
    biorbd::utils::Scalar Ta = exp( -(*m_qopt - pos) * (*m_qopt - pos)   /
                                    (2 * *m_r * *m_r)   );

    // Calculation of the max torque
    return Tw * A * Ta;

}

void biorbd::actuator::ActuatorGauss3p::setType()
{
    *m_type = biorbd::actuator::TYPE::GAUSS3P;
}
