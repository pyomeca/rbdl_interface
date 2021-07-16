#define BIORBD_API_EXPORTS
#include "Actuators/ActuatorGauss6p.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"

biorbd::actuator::ActuatorGauss6p::ActuatorGauss6p() :
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
    m_qopt(std::make_shared<biorbd::utils::Scalar>(0)),
    m_facteur(std::make_shared<biorbd::utils::Scalar>(0)),
    m_r2(std::make_shared<biorbd::utils::Scalar>(0)),
    m_qopt2(std::make_shared<biorbd::utils::Scalar>(0))
{
    setType();
}

biorbd::actuator::ActuatorGauss6p::ActuatorGauss6p(
    const biorbd::actuator::ActuatorGauss6p &other) :
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
    m_qopt(other.m_qopt),
    m_facteur(other.m_facteur),
    m_r2(other.m_r2),
    m_qopt2(other.m_qopt2)
{

}
biorbd::actuator::ActuatorGauss6p::ActuatorGauss6p(
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
    const biorbd::utils::Scalar& facteur,
    const biorbd::utils::Scalar& r2,
    const biorbd::utils::Scalar& qopt2,
    unsigned int dofIdx) :
    Actuator(direction, dofIdx),
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
    m_qopt(std::make_shared<biorbd::utils::Scalar>(qopt)),
    m_facteur(std::make_shared<biorbd::utils::Scalar>(facteur)),
    m_r2(std::make_shared<biorbd::utils::Scalar>(r2)),
    m_qopt2(std::make_shared<biorbd::utils::Scalar>(qopt2))
{
    setType();
}

biorbd::actuator::ActuatorGauss6p::ActuatorGauss6p(
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
    const biorbd::utils::Scalar& facteur,
    const biorbd::utils::Scalar& r2,
    const biorbd::utils::Scalar& qopt2,
    unsigned int dofIdx,
    const biorbd::utils::String &jointName) :
    Actuator(direction, dofIdx, jointName),
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
    m_qopt(std::make_shared<biorbd::utils::Scalar>(qopt)),
    m_facteur(std::make_shared<biorbd::utils::Scalar>(facteur)),
    m_r2(std::make_shared<biorbd::utils::Scalar>(r2)),
    m_qopt2(std::make_shared<biorbd::utils::Scalar>(qopt2))
{
    setType();
}

biorbd::actuator::ActuatorGauss6p::~ActuatorGauss6p()
{

}

biorbd::actuator::ActuatorGauss6p biorbd::actuator::ActuatorGauss6p::DeepCopy()
const
{
    biorbd::actuator::ActuatorGauss6p copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::actuator::ActuatorGauss6p::DeepCopy(
    const biorbd::actuator::ActuatorGauss6p &other)
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
    *m_facteur = *other.m_facteur;
    *m_r2 = *other.m_r2;
    *m_qopt2 = *other.m_qopt2;
}

biorbd::utils::Scalar biorbd::actuator::ActuatorGauss6p::torqueMax()
{
    biorbd::utils::Error::raise("torqueMax for ActuatorGauss6p must be called with Q and Qdot");
}

biorbd::utils::Scalar biorbd::actuator::ActuatorGauss6p::torqueMax(
    const biorbd::rigidbody::GeneralizedCoordinates &Q,
    const biorbd::rigidbody::GeneralizedVelocity &Qdot)
{
    biorbd::utils::Scalar pos(Q[*m_dofIdx] * 180/M_PI);
    biorbd::utils::Scalar speed(Qdot[*m_dofIdx] * 180/M_PI);

    // Tetanic torque max
    biorbd::utils::Scalar Tc = *m_T0 * *m_wc / *m_wmax;
    biorbd::utils::Scalar C = Tc*(*m_wmax + *m_wc); // concentric
    biorbd::utils::Scalar we = ( (*m_Tmax - *m_T0) * *m_wmax * *m_wc )   /
                               ( *m_k * *m_T0 * (*m_wmax + *m_wc) );
    biorbd::utils::Scalar E = -( *m_Tmax - *m_T0 ) * we; // eccentric

    biorbd::utils::Scalar Tw;
#ifdef BIORBD_USE_CASADI_MATH
    Tw = casadi::MX::if_else(casadi::MX::ge(speed, 0),
                             C / ( *m_wc + speed )  - Tc,
                             E / ( we - speed ) + *m_Tmax);
#else
    if (speed >= 0) {
        Tw = C / ( *m_wc + speed )  - Tc;    // For the concentric
    } else {
        Tw = E / ( we - speed ) + *m_Tmax;    // For the eccentric
    }
#endif

    // Differential activation
    biorbd::utils::Scalar A =
        *m_amin + ( *m_amax - *m_amin )
        / ( 1 + biorbd::utils::Scalar(exp( -(speed - *m_w1) / *m_wr   )) );

    // Torque angle
    biorbd::utils::Scalar Ta = biorbd::utils::Scalar(exp( -(*m_qopt - pos) *
                               (*m_qopt - pos)  /  (2* *m_r * *m_r )   ))
                               + *m_facteur * biorbd::utils::Scalar(exp( -(*m_qopt2 - pos) *
                                       (*m_qopt2 - pos)  /  (2 * *m_r2 * *m_r2)   ));

    // Calculation of the max torque
    return Tw * A * Ta;
}

void biorbd::actuator::ActuatorGauss6p::setType()
{
    *m_type = biorbd::actuator::TYPE::GAUSS6P;
}
