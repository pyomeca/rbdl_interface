#define BIORBD_API_EXPORTS
#include "Actuators/Actuators.h"

#include <vector>
#include "Utils/Error.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/Joints.h"
#include "Actuators/Actuator.h"
#include "Actuators/ActuatorGauss3p.h"
#include "Actuators/ActuatorGauss6p.h"
#include "Actuators/ActuatorSigmoidGauss3p.h"
#include "Actuators/ActuatorConstant.h"
#include "Actuators/ActuatorLinear.h"

biorbd::actuator::Actuators::Actuators() :
    m_all(std::make_shared<std::vector<std::pair<std::shared_ptr<biorbd::actuator::Actuator>, std::shared_ptr<biorbd::actuator::Actuator>>>>()),
    m_isDofSet(std::make_shared<std::vector<bool>>(1)),
    m_isClose(std::make_shared<bool>(false))
{
    (*m_isDofSet)[0] = false;
}

biorbd::actuator::Actuators::Actuators(
    const biorbd::actuator::Actuators& other) :
    m_all(other.m_all),
    m_isDofSet(other.m_isDofSet),
    m_isClose(other.m_isClose)
{

}


biorbd::actuator::Actuators::~Actuators()
{

}

void biorbd::actuator::Actuators::DeepCopy(const biorbd::actuator::Actuators
        &other)
{
    m_all->resize(other.m_all->size());
    for (unsigned int i=0; i<other.m_all->size(); ++i) {
        if ((*other.m_all)[i].first->type() == biorbd::actuator::TYPE::CONSTANT)
            (*m_all)[i].first = std::make_shared<biorbd::actuator::ActuatorConstant>(
                                    static_cast<const biorbd::actuator::ActuatorConstant&>( *
                                            (*other.m_all)[i].first) );
        else if ((*other.m_all)[i].first->type() == biorbd::actuator::TYPE::LINEAR)
            (*m_all)[i].first = std::make_shared<biorbd::actuator::ActuatorLinear>(
                                    static_cast<const biorbd::actuator::ActuatorLinear&>( *
                                            (*other.m_all)[i].first) );
        else if ((*other.m_all)[i].first->type() == biorbd::actuator::TYPE::GAUSS3P)
            (*m_all)[i].first = std::make_shared<biorbd::actuator::ActuatorGauss3p>(
                                    static_cast<const biorbd::actuator::ActuatorGauss3p&>( *
                                            (*other.m_all)[i].first) );
        else if ((*other.m_all)[i].first->type() == biorbd::actuator::TYPE::GAUSS6P)
            (*m_all)[i].first = std::make_shared<biorbd::actuator::ActuatorGauss6p>(
                                    static_cast<const biorbd::actuator::ActuatorGauss6p&>( *
                                            (*other.m_all)[i].first) );
        else if ((*other.m_all)[i].first->type() ==
                 biorbd::actuator::TYPE::SIGMOIDGAUSS3P)
            (*m_all)[i].first = std::make_shared<biorbd::actuator::ActuatorSigmoidGauss3p>(
                                    static_cast<const biorbd::actuator::ActuatorSigmoidGauss3p&>( *
                                            (*other.m_all)[i].first) );
        else
            biorbd::utils::Error::raise("Actuator " + biorbd::utils::String(
                                            biorbd::actuator::TYPE_toStr((*other.m_all)[i].first->type()))
                                        + " in DeepCopy");
        if ((*other.m_all)[i].second->type() == biorbd::actuator::TYPE::CONSTANT)
            (*m_all)[i].second = std::make_shared<biorbd::actuator::ActuatorConstant>(
                                     static_cast<const biorbd::actuator::ActuatorConstant&>( *
                                             (*other.m_all)[i].second) );
        else if ((*other.m_all)[i].second->type() == biorbd::actuator::TYPE::LINEAR)
            (*m_all)[i].second = std::make_shared<biorbd::actuator::ActuatorLinear>(
                                     static_cast<const biorbd::actuator::ActuatorLinear&>( *
                                             (*other.m_all)[i].second) );
        else if ((*other.m_all)[i].second->type() == biorbd::actuator::TYPE::GAUSS3P)
            (*m_all)[i].second = std::make_shared<biorbd::actuator::ActuatorGauss3p>(
                                     static_cast<const biorbd::actuator::ActuatorGauss3p&>( *
                                             (*other.m_all)[i].second) );
        else if ((*other.m_all)[i].second->type() == biorbd::actuator::TYPE::GAUSS6P)
            (*m_all)[i].second = std::make_shared<biorbd::actuator::ActuatorGauss6p>(
                                     static_cast<const biorbd::actuator::ActuatorGauss6p&>( *
                                             (*other.m_all)[i].second) );
        else if ((*other.m_all)[i].second->type() ==
                 biorbd::actuator::TYPE::SIGMOIDGAUSS3P)
            (*m_all)[i].second = std::make_shared<biorbd::actuator::ActuatorSigmoidGauss3p>(
                                     static_cast<const biorbd::actuator::ActuatorSigmoidGauss3p&>( *
                                             (*other.m_all)[i].second) );
        else
            biorbd::utils::Error::raise("Actuator " + biorbd::utils::String(
                                            biorbd::actuator::TYPE_toStr((*other.m_all)[i].second->type()))
                                        + " in DeepCopy");
    }
    m_isDofSet->resize(other.m_isDofSet->size());
    for (unsigned int i=0; i<other.m_isDofSet->size(); ++i) {
        (*m_isDofSet)[i] = (*other.m_isDofSet)[i];
    }
    *m_isClose = *other.m_isClose;
}

void biorbd::actuator::Actuators::addActuator(const biorbd::actuator::Actuator
        &act)
{
    biorbd::utils::Error::check(
        !*m_isClose, "You can't add actuator after closing the model");

    // Assuming that this is also a Joints type (via BiorbdModel)
    const biorbd::rigidbody::Joints &model =
        dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    // Verify that the target dof is associated to a dof that
    // already exists in the model
    biorbd::utils::Error::check(
        act.index()<model.nbDof(), "Sent index is out of dof range");

    // For speed purposes and coherence with the Q,
    // set the actuator to the same index as its associated dof
    unsigned int idx(act.index());

    // If there are less actuators declared than dof,
    // the vector must be enlarged
    if (idx >= m_all->size()) {
        m_all->resize(idx+1);
        m_isDofSet->resize((idx+1)*2, false);
    }

    // Add an actuator to the pool of actuators according to its type
    if (act.type() == biorbd::actuator::TYPE::CONSTANT) {
        if (act.direction() == 1) {
            (*m_all)[idx].first = std::make_shared<biorbd::actuator::ActuatorConstant>
                                  (static_cast<const biorbd::actuator::ActuatorConstant&>(act));
            (*m_isDofSet)[idx*2] = true;
        } else {
            (*m_all)[idx].second = std::make_shared<biorbd::actuator::ActuatorConstant>
                                   (static_cast<const biorbd::actuator::ActuatorConstant&>(act));
            (*m_isDofSet)[idx*2+1] = true;
        }
        return;
    } else if (act.type() == biorbd::actuator::TYPE::LINEAR) {
        if (act.direction() == 1) {
            (*m_all)[idx].first = std::make_shared<biorbd::actuator::ActuatorLinear>
                                  (static_cast<const biorbd::actuator::ActuatorLinear&>(act));
            (*m_isDofSet)[idx*2] = true;
        } else {
            (*m_all)[idx].second = std::make_shared<biorbd::actuator::ActuatorLinear>
                                   (static_cast<const biorbd::actuator::ActuatorLinear&>(act));
            (*m_isDofSet)[idx*2+1] = true;
        }
        return;
    } else if (act.type() == biorbd::actuator::TYPE::GAUSS3P) {
        if (act.direction() == 1) {
            (*m_all)[idx].first = std::make_shared<biorbd::actuator::ActuatorGauss3p>
                                  (static_cast<const biorbd::actuator::ActuatorGauss3p&>(act));
            (*m_isDofSet)[idx*2] = true;
        } else {
            (*m_all)[idx].second = std::make_shared<biorbd::actuator::ActuatorGauss3p>
                                   (static_cast<const biorbd::actuator::ActuatorGauss3p&>(act));
            (*m_isDofSet)[idx*2+1] = true;
        }
        return;
    } else if (act.type() == biorbd::actuator::TYPE::GAUSS6P) {
        if (act.direction() == 1) {
            (*m_all)[idx].first = std::make_shared<biorbd::actuator::ActuatorGauss6p>
                                  (static_cast<const biorbd::actuator::ActuatorGauss6p&>(act));
            (*m_isDofSet)[idx*2] = true;
        } else {
            (*m_all)[idx].second = std::make_shared<biorbd::actuator::ActuatorGauss6p>
                                   (static_cast<const biorbd::actuator::ActuatorGauss6p&>(act));
            (*m_isDofSet)[idx*2+1] = true;
        }
        return;
    } else if (act.type() == biorbd::actuator::TYPE::SIGMOIDGAUSS3P) {
        if (act.direction() == 1) {
            (*m_all)[idx].first = std::make_shared<biorbd::actuator::ActuatorSigmoidGauss3p>
                                  (static_cast<const biorbd::actuator::ActuatorSigmoidGauss3p&>(act));
            (*m_isDofSet)[idx*2] = true;
        } else {
            (*m_all)[idx].second =
                std::make_shared<biorbd::actuator::ActuatorSigmoidGauss3p>
                (static_cast<const biorbd::actuator::ActuatorSigmoidGauss3p&>(act));
            (*m_isDofSet)[idx*2+1] = true;
        }
        return;
    } else {
        biorbd::utils::Error::raise("Actuator type not found");
    }

}

void biorbd::actuator::Actuators::closeActuator()
{
    // Assuming that this is also a Joints type (via BiorbdModel)
    const biorbd::rigidbody::Joints &model =
        dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    biorbd::utils::Error::check(
        model.nbDof()==m_all->size(),
        "All dof must have their actuators set");

    for (unsigned int i=0; i<m_all->size()*2; ++i)
        biorbd::utils::Error::check((*m_isDofSet)[i],
                                    "All DoF must have their actuators set "
                                    "before closing the model");

    *m_isClose = true;
}

const std::pair<std::shared_ptr<biorbd::actuator::Actuator>,
      std::shared_ptr<biorbd::actuator::Actuator>>&
      biorbd::actuator::Actuators::actuator(unsigned int dof)
{
    biorbd::utils::Error::check(dof<nbActuators(),
                                "Idx asked is higher than number of actuator");
    return (*m_all)[dof];
}
const biorbd::actuator::Actuator& biorbd::actuator::Actuators::actuator(
    unsigned int dof,
    bool concentric)
{
    const std::pair<std::shared_ptr<biorbd::actuator::Actuator>, std::shared_ptr<biorbd::actuator::Actuator>>&
            tp(actuator(dof));

    if (concentric) {
        return *tp.first;
    } else {
        return *tp.second;
    }
}

unsigned int biorbd::actuator::Actuators::nbActuators() const
{
    return static_cast<unsigned int>(m_all->size());
}

biorbd::rigidbody::GeneralizedTorque biorbd::actuator::Actuators::torque(
    const biorbd::utils::Vector& activation,
    const biorbd::rigidbody::GeneralizedCoordinates& Q,
    const biorbd::rigidbody::GeneralizedVelocity &Qdot)
{
    // Calculate the maximal torques
    biorbd::rigidbody::GeneralizedTorque GeneralizedTorque(
        torqueMax(activation,Q,Qdot));

    // Put the signs
    for (unsigned int i=0; i<GeneralizedTorque.size(); ++i) {
        GeneralizedTorque(i) = GeneralizedTorque(i) * activation(i);
    }

    return GeneralizedTorque;
}


std::pair<biorbd::rigidbody::GeneralizedTorque, biorbd::rigidbody::GeneralizedTorque>
biorbd::actuator::Actuators::torqueMax(
    const biorbd::rigidbody::GeneralizedCoordinates& Q,
    const biorbd::rigidbody::GeneralizedVelocity& Qdot)
{
    biorbd::utils::Error::check(*m_isClose,
                                "Close the actuator model before calling torqueMax");

    // Assuming that this is also a Joints type (via BiorbdModel)
    const biorbd::rigidbody::Joints &model =
        dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    std::pair<biorbd::rigidbody::GeneralizedTorque, biorbd::rigidbody::GeneralizedTorque>
    maxGeneralizedTorque_all =
        std::make_pair(biorbd::rigidbody::GeneralizedTorque(model),
                       biorbd::rigidbody::GeneralizedTorque(model));

    for (unsigned int i=0; i<model.nbDof(); ++i) {
        std::pair<std::shared_ptr<Actuator>, std::shared_ptr<Actuator>>
                GeneralizedTorque_tp(actuator(i));
        for (unsigned p=0; p<2; ++p) {
            if (p==0) // First
                if (std::dynamic_pointer_cast<ActuatorGauss3p> (GeneralizedTorque_tp.first)) {
                    maxGeneralizedTorque_all.first[i] = std::static_pointer_cast<ActuatorGauss3p>
                                                        (GeneralizedTorque_tp.first)->torqueMax(Q,
                                                                Qdot);
                } else if (std::dynamic_pointer_cast<ActuatorConstant>
                           (GeneralizedTorque_tp.first)) {
                    maxGeneralizedTorque_all.first[i] = std::static_pointer_cast<ActuatorConstant>
                                                        (GeneralizedTorque_tp.first)->torqueMax();
                } else if (std::dynamic_pointer_cast<ActuatorLinear>
                           (GeneralizedTorque_tp.first)) {
                    maxGeneralizedTorque_all.first[i] = std::static_pointer_cast<ActuatorLinear>
                                                        (GeneralizedTorque_tp.first)->torqueMax(
                                                            Q);
                } else if (std::dynamic_pointer_cast<ActuatorGauss6p>
                           (GeneralizedTorque_tp.first)) {
                    maxGeneralizedTorque_all.first[i] = std::static_pointer_cast<ActuatorGauss6p>
                                                        (GeneralizedTorque_tp.first)->torqueMax(Q,
                                                                Qdot);
                } else if (std::dynamic_pointer_cast<ActuatorSigmoidGauss3p>
                           (GeneralizedTorque_tp.first)) {
                    maxGeneralizedTorque_all.first[i] =
                        std::static_pointer_cast<ActuatorSigmoidGauss3p>
                        (GeneralizedTorque_tp.first)->torqueMax(Q, Qdot);
                } else {
                    biorbd::utils::Error::raise("Wrong type (should never get here because of previous safety)");
                }
            else // Second
                if (std::dynamic_pointer_cast<ActuatorGauss3p> (GeneralizedTorque_tp.second)) {
                    maxGeneralizedTorque_all.second[i] = std::static_pointer_cast<ActuatorGauss3p>
                                                         (GeneralizedTorque_tp.second)->torqueMax(
                                                                 Q, Qdot);
                } else if (std::dynamic_pointer_cast<ActuatorConstant>
                           (GeneralizedTorque_tp.second)) {
                    maxGeneralizedTorque_all.second[i] = std::static_pointer_cast<ActuatorConstant>
                                                         (GeneralizedTorque_tp.second)->torqueMax();
                } else if (std::dynamic_pointer_cast<ActuatorLinear>
                           (GeneralizedTorque_tp.second)) {
                    maxGeneralizedTorque_all.second[i] = std::static_pointer_cast<ActuatorLinear>
                                                         (GeneralizedTorque_tp.second)->torqueMax(
                                                                 Q);
                } else if (std::dynamic_pointer_cast<ActuatorGauss6p>
                           (GeneralizedTorque_tp.second)) {
                    maxGeneralizedTorque_all.second[i] = std::static_pointer_cast<ActuatorGauss6p>
                                                         (GeneralizedTorque_tp.second)->torqueMax(
                                                                 Q, Qdot);
                } else if (std::dynamic_pointer_cast<ActuatorSigmoidGauss3p>
                           (GeneralizedTorque_tp.second)) {
                    maxGeneralizedTorque_all.second[i] =
                        std::static_pointer_cast<ActuatorSigmoidGauss3p>
                        (GeneralizedTorque_tp.second)->torqueMax(Q, Qdot);
                } else {
                    biorbd::utils::Error::raise("Wrong type (should never get here because of previous safety)");
                }
        }
    }

    return maxGeneralizedTorque_all;
}


biorbd::rigidbody::GeneralizedTorque biorbd::actuator::Actuators::torqueMax(
    const utils::Vector &activation,
    const biorbd::rigidbody::GeneralizedCoordinates& Q,
    const biorbd::rigidbody::GeneralizedVelocity &Qdot)
{
    biorbd::utils::Error::check(*m_isClose,
                                "Close the actuator model before calling torqueMax");

    // Assuming that this is also a Joints type (via BiorbdModel)
    const biorbd::rigidbody::Joints &model =
        dynamic_cast<biorbd::rigidbody::Joints &>(*this);

    // Set qdot to be positive if concentric and negative if excentric
    biorbd::rigidbody::GeneralizedVelocity QdotResigned(Qdot);
    for (unsigned int i=0; i<Qdot.size(); ++i) {
#ifdef BIORBD_USE_CASADI_MATH
        QdotResigned(i) = casadi::MX::if_else(
                              casadi::MX::lt(activation(i), 0),
                              -Qdot(i), Qdot(i));
#else
        if (activation(i)<0) {
            QdotResigned(i) = -Qdot(i);
        }
#endif
    }

    biorbd::rigidbody::GeneralizedTorque maxGeneralizedTorque_all(model);

    for (unsigned int i=0; i<model.nbDof(); ++i) {
#ifdef BIORBD_USE_CASADI_MATH
        maxGeneralizedTorque_all[i] = casadi::MX::if_else(
                                          casadi::MX::ge(activation(i, 0), 0),
                                          getTorqueMaxDirection(actuator(i).first, Q, QdotResigned),
                                          getTorqueMaxDirection(actuator(i).second, Q, QdotResigned));
#else
        if (activation[i] >= 0) { // First
            maxGeneralizedTorque_all[i] = getTorqueMaxDirection(actuator(i).first, Q,
                                          QdotResigned);
        } else {
            maxGeneralizedTorque_all[i] = getTorqueMaxDirection(actuator(i).second, Q,
                                          QdotResigned);
        }
#endif
    }

    return maxGeneralizedTorque_all;
}

biorbd::utils::Scalar biorbd::actuator::Actuators::getTorqueMaxDirection(
    const std::shared_ptr<biorbd::actuator::Actuator> actuator,
    const biorbd::rigidbody::GeneralizedCoordinates& Q,
    const biorbd::rigidbody::GeneralizedVelocity& Qdot) const
{
    if (std::dynamic_pointer_cast<ActuatorGauss3p> (actuator)) {
        return std::static_pointer_cast<ActuatorGauss3p> (actuator)->torqueMax(Q, Qdot);
    } else if (std::dynamic_pointer_cast<ActuatorConstant> (actuator)) {
        return std::static_pointer_cast<ActuatorConstant> (actuator)->torqueMax();
    } else if (std::dynamic_pointer_cast<ActuatorLinear> (actuator)) {
        return std::static_pointer_cast<ActuatorLinear> (actuator)->torqueMax(Q);
    } else if (std::dynamic_pointer_cast<ActuatorGauss6p> (actuator)) {
        return std::static_pointer_cast<ActuatorGauss6p> (actuator)->torqueMax(Q, Qdot);
    } else if (std::dynamic_pointer_cast<ActuatorSigmoidGauss3p> (actuator)) {
        return std::static_pointer_cast<ActuatorSigmoidGauss3p> (actuator)->torqueMax(Q,
                Qdot);
    } else {
        biorbd::utils::Error::raise("Wrong type (should never get here because of previous safety)");
    }

}
