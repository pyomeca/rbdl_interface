#define BIORBD_API_EXPORTS
#include "Muscles/HillType.h"

#include "Utils/Error.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "Muscles/Characteristics.h"
#include "Muscles/Geometry.h"
#include "Muscles/State.h"

biorbd::muscles::HillType::HillType() :
    biorbd::muscles::Muscle(),
    m_damping(std::make_shared<biorbd::utils::Scalar>()),
    m_FlCE(std::make_shared<biorbd::utils::Scalar>()),
    m_FlPE(std::make_shared<biorbd::utils::Scalar>()),
    m_FvCE(std::make_shared<biorbd::utils::Scalar>()),
    m_cste_FlCE_1(std::make_shared<biorbd::utils::Scalar>(0.15)),
    m_cste_FlCE_2(std::make_shared<biorbd::utils::Scalar>(0.45)),
    m_cste_FvCE_1(std::make_shared<biorbd::utils::Scalar>(1)),
    m_cste_FvCE_2(std::make_shared<biorbd::utils::Scalar>(-.33/2 * *m_cste_FvCE_1/
                  (1+*m_cste_FvCE_1))),
    m_cste_FlPE_1(std::make_shared<biorbd::utils::Scalar>(10.0)),
    m_cste_FlPE_2(std::make_shared<biorbd::utils::Scalar>(5.0)),
    m_cste_eccentricForceMultiplier(std::make_shared<biorbd::utils::Scalar>(1.8)),
    m_cste_damping(std::make_shared<biorbd::utils::Scalar>(0.1)),
    m_cste_maxShorteningSpeed(std::make_shared<biorbd::utils::Scalar>(10.0))
{
    setType();
}

biorbd::muscles::HillType::HillType(
    const biorbd::utils::String &name,
    const biorbd::muscles::Geometry &geometry,
    const biorbd::muscles::Characteristics &characteristics) :
    biorbd::muscles::Muscle(name,geometry,characteristics),
    m_damping(std::make_shared<biorbd::utils::Scalar>()),
    m_FlCE(std::make_shared<biorbd::utils::Scalar>()),
    m_FlPE(std::make_shared<biorbd::utils::Scalar>()),
    m_FvCE(std::make_shared<biorbd::utils::Scalar>()),
    m_cste_FlCE_1(std::make_shared<biorbd::utils::Scalar>(0.15)),
    m_cste_FlCE_2(std::make_shared<biorbd::utils::Scalar>(0.45)),
    m_cste_FvCE_1(std::make_shared<biorbd::utils::Scalar>(1)),
    m_cste_FvCE_2(std::make_shared<biorbd::utils::Scalar>(-.33/2 * *m_cste_FvCE_1/
                  (1+*m_cste_FvCE_1))),
    m_cste_FlPE_1(std::make_shared<biorbd::utils::Scalar>(10.0)),
    m_cste_FlPE_2(std::make_shared<biorbd::utils::Scalar>(5.0)),
    m_cste_eccentricForceMultiplier(std::make_shared<biorbd::utils::Scalar>(1.8)),
    m_cste_damping(std::make_shared<biorbd::utils::Scalar>(0.1)),
    m_cste_maxShorteningSpeed(std::make_shared<biorbd::utils::Scalar>(10.0))
{
    setType();
}

biorbd::muscles::HillType::HillType(
    const biorbd::utils::String &name,
    const biorbd::muscles::Geometry &geometry,
    const biorbd::muscles::Characteristics &characteristics,
    const biorbd::muscles::State& emg) :
    biorbd::muscles::Muscle(name,geometry,characteristics, emg),
    m_damping(std::make_shared<biorbd::utils::Scalar>()),
    m_FlCE(std::make_shared<biorbd::utils::Scalar>()),
    m_FlPE(std::make_shared<biorbd::utils::Scalar>()),
    m_FvCE(std::make_shared<biorbd::utils::Scalar>()),
    m_cste_FlCE_1(std::make_shared<biorbd::utils::Scalar>(0.15)),
    m_cste_FlCE_2(std::make_shared<biorbd::utils::Scalar>(0.45)),
    m_cste_FvCE_1(std::make_shared<biorbd::utils::Scalar>(1)),
    m_cste_FvCE_2(std::make_shared<biorbd::utils::Scalar>(-.33/2 * *m_cste_FvCE_1/
                  (1+*m_cste_FvCE_1))),
    m_cste_FlPE_1(std::make_shared<biorbd::utils::Scalar>(10.0)),
    m_cste_FlPE_2(std::make_shared<biorbd::utils::Scalar>(5.0)),
    m_cste_eccentricForceMultiplier(std::make_shared<biorbd::utils::Scalar>(1.8)),
    m_cste_damping(std::make_shared<biorbd::utils::Scalar>(0.1)),
    m_cste_maxShorteningSpeed(std::make_shared<biorbd::utils::Scalar>(10.0))
{
    setType();
}

biorbd::muscles::HillType::HillType(
    const biorbd::utils::String &name,
    const biorbd::muscles::Geometry &geometry,
    const biorbd::muscles::Characteristics &characteristics,
    const biorbd::muscles::PathModifiers &pathModifiers) :
    biorbd::muscles::Muscle(name,geometry,characteristics,pathModifiers),
    m_damping(std::make_shared<biorbd::utils::Scalar>()),
    m_FlCE(std::make_shared<biorbd::utils::Scalar>()),
    m_FlPE(std::make_shared<biorbd::utils::Scalar>()),
    m_FvCE(std::make_shared<biorbd::utils::Scalar>()),
    m_cste_FlCE_1(std::make_shared<biorbd::utils::Scalar>(0.15)),
    m_cste_FlCE_2(std::make_shared<biorbd::utils::Scalar>(0.45)),
    m_cste_FvCE_1(std::make_shared<biorbd::utils::Scalar>(1)),
    m_cste_FvCE_2(std::make_shared<biorbd::utils::Scalar>(-.33/2 * *m_cste_FvCE_1/
                  (1+*m_cste_FvCE_1))),
    m_cste_FlPE_1(std::make_shared<biorbd::utils::Scalar>(10.0)),
    m_cste_FlPE_2(std::make_shared<biorbd::utils::Scalar>(5.0)),
    m_cste_eccentricForceMultiplier(std::make_shared<biorbd::utils::Scalar>(1.8)),
    m_cste_damping(std::make_shared<biorbd::utils::Scalar>(0.1)),
    m_cste_maxShorteningSpeed(std::make_shared<biorbd::utils::Scalar>(10.0))
{
    setType();
}
biorbd::muscles::HillType::HillType(
    const biorbd::utils::String& name,
    const biorbd::muscles::Geometry& geometry,
    const biorbd::muscles::Characteristics& characteristics,
    const biorbd::muscles::PathModifiers &pathModifiers,
    const biorbd::muscles::State& state) :
    biorbd::muscles::Muscle(name,geometry,characteristics,pathModifiers,state),
    m_damping(std::make_shared<biorbd::utils::Scalar>()),
    m_FlCE(std::make_shared<biorbd::utils::Scalar>()),
    m_FlPE(std::make_shared<biorbd::utils::Scalar>()),
    m_FvCE(std::make_shared<biorbd::utils::Scalar>()),
    m_cste_FlCE_1(std::make_shared<biorbd::utils::Scalar>(0.15)),
    m_cste_FlCE_2(std::make_shared<biorbd::utils::Scalar>(0.45)),
    m_cste_FvCE_1(std::make_shared<biorbd::utils::Scalar>(1)),
    m_cste_FvCE_2(std::make_shared<biorbd::utils::Scalar>(-.33/2 * *m_cste_FvCE_1/
                  (1+*m_cste_FvCE_1))),
    m_cste_FlPE_1(std::make_shared<biorbd::utils::Scalar>(10.0)),
    m_cste_FlPE_2(std::make_shared<biorbd::utils::Scalar>(5.0)),
    m_cste_eccentricForceMultiplier(std::make_shared<biorbd::utils::Scalar>(1.8)),
    m_cste_damping(std::make_shared<biorbd::utils::Scalar>(0.1)),
    m_cste_maxShorteningSpeed(std::make_shared<biorbd::utils::Scalar>(10.0))
{
    setType();
}

biorbd::muscles::HillType::HillType(const biorbd::muscles::Muscle &other) :
    biorbd::muscles::Muscle (other)
{
    const biorbd::muscles::HillType & m_tp(
        dynamic_cast<const biorbd::muscles::HillType &>(other));
    m_damping = m_tp.m_damping;
    m_FlCE = m_tp.m_FlCE;
    m_FlPE = m_tp.m_FlPE;
    m_FvCE = m_tp.m_FvCE;
    m_cste_FlCE_1 = m_tp.m_cste_FlCE_1;
    m_cste_FlCE_2 = m_tp.m_cste_FlCE_2;
    m_cste_FvCE_1 = m_tp.m_cste_FvCE_1;
    m_cste_FvCE_2 = m_tp.m_cste_FvCE_2;
    m_cste_FlPE_1 = m_tp.m_cste_FlPE_1;
    m_cste_FlPE_2 = m_tp.m_cste_FlPE_2;
    m_cste_eccentricForceMultiplier = m_tp.m_cste_eccentricForceMultiplier;
    m_cste_damping = m_tp.m_cste_damping;
    m_cste_maxShorteningSpeed = m_tp.m_cste_maxShorteningSpeed;
}

biorbd::muscles::HillType::HillType(
    const std::shared_ptr<biorbd::muscles::Muscle> other) :
    biorbd::muscles::Muscle (other)
{
    const std::shared_ptr<biorbd::muscles::HillType> m_tp(
        std::dynamic_pointer_cast<biorbd::muscles::HillType>(other));
    biorbd::utils::Error::check(m_tp != nullptr, "Muscle must be of a Hill Type");
    m_damping = m_tp->m_damping;
    m_FlCE = m_tp->m_FlCE;
    m_FlPE = m_tp->m_FlPE;
    m_FvCE = m_tp->m_FvCE;
    m_cste_FlCE_1 = m_tp->m_cste_FlCE_1;
    m_cste_FlCE_2 = m_tp->m_cste_FlCE_2;
    m_cste_FvCE_1 = m_tp->m_cste_FvCE_1;
    m_cste_FvCE_2 = m_tp->m_cste_FvCE_2;
    m_cste_FlPE_1 = m_tp->m_cste_FlPE_1;
    m_cste_FlPE_2 = m_tp->m_cste_FlPE_2;
    m_cste_eccentricForceMultiplier = m_tp->m_cste_eccentricForceMultiplier;
    m_cste_damping = m_tp->m_cste_damping;
    m_cste_maxShorteningSpeed = m_tp->m_cste_maxShorteningSpeed;
}

biorbd::muscles::HillType biorbd::muscles::HillType::DeepCopy() const
{
    biorbd::muscles::HillType copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::HillType::DeepCopy(const biorbd::muscles::HillType &other)
{
    biorbd::muscles::Muscle::DeepCopy(other);
    *m_damping = *other.m_damping;
    *m_FlCE = *other.m_FlCE;
    *m_FlPE = *other.m_FlPE;
    *m_FvCE = *other.m_FvCE;
    *m_cste_FlCE_1 = *other.m_cste_FlCE_1;
    *m_cste_FlCE_2 = *other.m_cste_FlCE_2;
    *m_cste_FvCE_1 = *other.m_cste_FvCE_1;
    *m_cste_FvCE_2 = *other.m_cste_FvCE_2;
    *m_cste_FlPE_1 = *other.m_cste_FlPE_1;
    *m_cste_FlPE_2 = *other.m_cste_FlPE_2;
    *m_cste_eccentricForceMultiplier = *other.m_cste_eccentricForceMultiplier;
    *m_cste_damping = *other.m_cste_damping;
    *m_cste_maxShorteningSpeed = *other.m_cste_maxShorteningSpeed;
}

const biorbd::utils::Scalar& biorbd::muscles::HillType::force(
    const biorbd::muscles::State& emg)
{
    // Compute the forces of each element
    computeFvCE();
    computeFlCE(emg);
    computeFlPE();
    computeDamping();

    // Combine the forces
    computeForce(emg);
    return *m_force;
}

const biorbd::utils::Scalar& biorbd::muscles::HillType::force(
    biorbd::rigidbody::Joints &model,
    const biorbd::rigidbody::GeneralizedCoordinates &Q,
    const biorbd::rigidbody::GeneralizedVelocity &Qdot,
    const biorbd::muscles::State &emg,
    int updateKin)
{
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = 2;
#endif
    // Update the configuration
    if (updateKin == 1) {
        updateOrientations(model,Q,Qdot,false);
    } else if (updateKin == 2) {
        updateOrientations(model,Q,Qdot,2);
    } else {
        biorbd::utils::Error::check(updateKin == 0,
                                    "Wrong level of update in force function");
    }

    // Computation
    return force(emg);
}

const biorbd::utils::Scalar& biorbd::muscles::HillType::force(
    biorbd::rigidbody::Joints &,
    const biorbd::rigidbody::GeneralizedCoordinates &,
    const biorbd::muscles::State &,
    int)
{
    biorbd::utils::Error::raise("Hill type needs velocity");
#ifdef _WIN32
    return *m_force; // Will never reach here
#endif
}

const biorbd::utils::Scalar& biorbd::muscles::HillType::FlCE(
    const biorbd::muscles::State &EMG)
{
    computeFlCE(EMG);
    return *m_FlCE;
}

const biorbd::utils::Scalar& biorbd::muscles::HillType::FlPE()
{
    computeFlPE();
    return *m_FlPE;
}

const biorbd::utils::Scalar& biorbd::muscles::HillType::FvCE()
{
    computeFvCE();
    return *m_FvCE;
}

const biorbd::utils::Scalar& biorbd::muscles::HillType::damping()
{
    computeDamping();
    return *m_damping;
}

void biorbd::muscles::HillType::setType()
{
    *m_type = biorbd::muscles::MUSCLE_TYPE::HILL;
}

void biorbd::muscles::HillType::computeDamping()
{
    *m_damping = position().velocity()
                 /
                 (*m_cste_maxShorteningSpeed * m_characteristics->optimalLength()) *
                 *m_cste_damping;
}

void biorbd::muscles::HillType::computeFlCE(const biorbd::muscles::State& emg)
{
    *m_FlCE = exp( -pow(( position().length() /
                          m_characteristics->optimalLength() / (*m_cste_FlCE_1*
                                  (1-emg.activation())+1) -1 ), 2)
                   /
                   *m_cste_FlCE_2   );
}

void biorbd::muscles::HillType::computeFvCE()
{
    // The relation is different if velocity< 0  or > 0
    biorbd::utils::Scalar v = m_position->velocity();
#ifdef BIORBD_USE_CASADI_MATH
    *m_FvCE = casadi::MX::if_else(
                  casadi::MX::le(v, 0),
                  (1.0-std::fabs(v) / *m_cste_maxShorteningSpeed) /
                  (1.0+std::fabs(v) / *m_cste_maxShorteningSpeed / *m_cste_FvCE_1),
                  (1.0-1.33*v / *m_cste_maxShorteningSpeed / *m_cste_FvCE_2) /
                  (1-v / *m_cste_maxShorteningSpeed / *m_cste_FvCE_2)
              );
#else
    if (v<=0)
        *m_FvCE = (1-fabs(v) / *m_cste_maxShorteningSpeed) /
                  (1+fabs(v) / *m_cste_maxShorteningSpeed / *m_cste_FvCE_1);
    else
        *m_FvCE = (1-1.33*v / *m_cste_maxShorteningSpeed / *m_cste_FvCE_2) /
                  (1-v / *m_cste_maxShorteningSpeed / *m_cste_FvCE_2);
#endif
}

void biorbd::muscles::HillType::computeFlPE()
{

#ifdef BIORBD_USE_CASADI_MATH
    *m_FlPE = casadi::MX::if_else_zero(
                  casadi::MX::gt(position().length(), characteristics().tendonSlackLength()),
                  exp(*m_cste_FlPE_1*(position().length()/characteristics().optimalLength()-1) -
                      *m_cste_FlPE_2));
#else
    if (position().length() > characteristics().tendonSlackLength()) {
        *m_FlPE = exp(*m_cste_FlPE_1*
                      (position().length()/characteristics().optimalLength()-1) - *m_cste_FlPE_2);
    } else {
        *m_FlPE = 0;
    }
#endif
}

biorbd::utils::Scalar biorbd::muscles::HillType::getForceFromActivation(
    const biorbd::muscles::State &emg)
{
    // Fonction qui permet de modifier la facon dont la multiplication est faite dans computeForce(EMG)
    return characteristics().forceIsoMax() * (emg.activation() * *m_FlCE * *m_FvCE +
            *m_FlPE + *m_damping);
}

void biorbd::muscles::HillType::normalizeEmg(
    biorbd::muscles::State& emg)
{
    emg.normalizeExcitation(characteristics().stateMax());
}
