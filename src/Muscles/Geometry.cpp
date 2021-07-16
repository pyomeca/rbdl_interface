#define BIORBD_API_EXPORTS
#include "Muscles/Geometry.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "Utils/RotoTrans.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/Joints.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "Muscles/WrappingObject.h"
#include "Muscles/PathModifiers.h"
#include "Muscles/Characteristics.h"
#include "Muscles/ViaPoint.h"
#include "Muscles/PathModifiers.h"

biorbd::muscles::Geometry::Geometry() :
    m_origin(std::make_shared<biorbd::utils::Vector3d>()),
    m_insertion(std::make_shared<biorbd::utils::Vector3d>()),
    m_originInGlobal(std::make_shared<biorbd::utils::Vector3d>
                     (biorbd::utils::Vector3d::Zero())),
    m_insertionInGlobal(std::make_shared<biorbd::utils::Vector3d>
                        (biorbd::utils::Vector3d::Zero())),
    m_pointsInGlobal(std::make_shared<std::vector<biorbd::utils::Vector3d>>()),
    m_pointsInLocal(std::make_shared<std::vector<biorbd::utils::Vector3d>>()),
    m_jacobian(std::make_shared<biorbd::utils::Matrix>()),
    m_G(std::make_shared<biorbd::utils::Matrix>()),
    m_jacobianLength(std::make_shared<biorbd::utils::Matrix>()),
    m_length(std::make_shared<biorbd::utils::Scalar>(0)),
    m_muscleTendonLength(std::make_shared<biorbd::utils::Scalar>(0)),
    m_velocity(std::make_shared<biorbd::utils::Scalar>(0)),
    m_isGeometryComputed(std::make_shared<bool>(false)),
    m_isVelocityComputed(std::make_shared<bool>(false)),
    m_posAndJacoWereForced(std::make_shared<bool>(false))
{

}

biorbd::muscles::Geometry::Geometry(
    const biorbd::utils::Vector3d &origin,
    const biorbd::utils::Vector3d &insertion) :
    m_origin(std::make_shared<biorbd::utils::Vector3d>(origin)),
    m_insertion(std::make_shared<biorbd::utils::Vector3d>(insertion)),
    m_originInGlobal(std::make_shared<biorbd::utils::Vector3d>
                     (biorbd::utils::Vector3d::Zero())),
    m_insertionInGlobal(std::make_shared<biorbd::utils::Vector3d>
                        (biorbd::utils::Vector3d::Zero())),
    m_pointsInGlobal(std::make_shared<std::vector<biorbd::utils::Vector3d>>()),
    m_pointsInLocal(std::make_shared<std::vector<biorbd::utils::Vector3d>>()),
    m_jacobian(std::make_shared<biorbd::utils::Matrix>()),
    m_G(std::make_shared<biorbd::utils::Matrix>()),
    m_jacobianLength(std::make_shared<biorbd::utils::Matrix>()),
    m_length(std::make_shared<biorbd::utils::Scalar>(0)),
    m_muscleTendonLength(std::make_shared<biorbd::utils::Scalar>(0)),
    m_velocity(std::make_shared<biorbd::utils::Scalar>(0)),
    m_isGeometryComputed(std::make_shared<bool>(false)),
    m_isVelocityComputed(std::make_shared<bool>(false)),
    m_posAndJacoWereForced(std::make_shared<bool>(false))
{

}

biorbd::muscles::Geometry biorbd::muscles::Geometry::DeepCopy() const
{
    biorbd::muscles::Geometry copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::muscles::Geometry::DeepCopy(const biorbd::muscles::Geometry &other)
{
    *m_origin = other.m_origin->DeepCopy();
    *m_insertion = other.m_insertion->DeepCopy();
    *m_originInGlobal = other.m_originInGlobal->DeepCopy();
    *m_insertionInGlobal = other.m_insertionInGlobal->DeepCopy();
    m_pointsInGlobal->resize(other.m_pointsInGlobal->size());
    for (unsigned int i=0; i<other.m_pointsInGlobal->size(); ++i) {
        (*m_pointsInGlobal)[i] = (*other.m_pointsInGlobal)[i].DeepCopy();
    }
    m_pointsInLocal->resize(other.m_pointsInLocal->size());
    for (unsigned int i=0; i<other.m_pointsInLocal->size(); ++i) {
        (*m_pointsInLocal)[i] = (*other.m_pointsInLocal)[i].DeepCopy();
    }
    *m_jacobian = *other.m_jacobian;
    *m_G = *other.m_G;
    *m_jacobianLength = *other.m_jacobianLength;
    *m_length = *other.m_length;
    *m_muscleTendonLength = *other.m_muscleTendonLength;
    *m_velocity = *other.m_velocity;
    *m_isGeometryComputed = *other.m_isGeometryComputed;
    *m_isVelocityComputed = *other.m_isVelocityComputed;
    *m_posAndJacoWereForced = *other.m_posAndJacoWereForced;
}


// ------ PUBLIC FUNCTIONS ------ //
void biorbd::muscles::Geometry::updateKinematics(
    biorbd::rigidbody::Joints &model,
    const biorbd::rigidbody::GeneralizedCoordinates *Q,
    const biorbd::rigidbody::GeneralizedVelocity *Qdot,
    int updateKin)
{
    if (*m_posAndJacoWereForced) {
        biorbd::utils::Error::warning(
            false,
            "Warning, using updateKinematics overrides the"
            " previously sent position and jacobian");
        *m_posAndJacoWereForced = false;
    }

    // Make sure the model is in the right configuration
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = 2;
#endif
    if (updateKin > 1) {
        model.UpdateKinematicsCustom(Q, Qdot, nullptr);
    }

    // Position of the points in space
    setMusclesPointsInGlobal(model, *Q);

    // Compute the Jacobian of the muscle points
    jacobian(model, *Q);

    // Complete the update
    _updateKinematics(Qdot);
}

void biorbd::muscles::Geometry::updateKinematics(biorbd::rigidbody::Joints
        &model,
        const biorbd::muscles::Characteristics& characteristics,
        biorbd::muscles::PathModifiers &pathModifiers,
        const biorbd::rigidbody::GeneralizedCoordinates *Q,
        const biorbd::rigidbody::GeneralizedVelocity *Qdot,
        int updateKin)
{
    if (*m_posAndJacoWereForced) {
        biorbd::utils::Error::warning(
            false, "Warning, using updateKinematics overrides the"
            " previously sent position and jacobian");
        *m_posAndJacoWereForced = false;
    }
#ifdef BIORBD_USE_CASADI_MATH
    updateKin = 2;
#endif

    // Ensure the model is in the right configuration
    if (updateKin > 1) {
        model.UpdateKinematicsCustom(Q, Qdot);
    }

    // Position of the points in space
    setMusclesPointsInGlobal(model, *Q, &pathModifiers);

    // Compute the Jacobian of the muscle points
    jacobian(model, *Q);

    // Complete the update
    _updateKinematics(Qdot, &characteristics, &pathModifiers);
}

void biorbd::muscles::Geometry::updateKinematics(
    std::vector<biorbd::utils::Vector3d>& musclePointsInGlobal,
    biorbd::utils::Matrix& jacoPointsInGlobal,
    const biorbd::rigidbody::GeneralizedVelocity* Qdot)
{
    *m_posAndJacoWereForced = true;

    // Position of the points in space
    setMusclesPointsInGlobal(musclePointsInGlobal);

    // Compute the Jacobian of the muscle points
    jacobian(jacoPointsInGlobal);

    // Complete the update
    _updateKinematics(Qdot);
}

void biorbd::muscles::Geometry::updateKinematics(
    std::vector<biorbd::utils::Vector3d>& musclePointsInGlobal,
    biorbd::utils::Matrix& jacoPointsInGlobal,
    const biorbd::muscles::Characteristics& c,
    const biorbd::rigidbody::GeneralizedVelocity* Qdot)
{
    *m_posAndJacoWereForced = true;

    // Position of the points in space
    setMusclesPointsInGlobal(musclePointsInGlobal);

    // Compute the Jacobian of the muscle points
    jacobian(jacoPointsInGlobal);

    // Complete the update
    _updateKinematics(Qdot, &c);
}

// Get and set the positions of the origins and insertions
void biorbd::muscles::Geometry::setOrigin(
    const utils::Vector3d &position)
{
    if (dynamic_cast<const biorbd::rigidbody::NodeSegment*>(&position)) {
        *m_origin = position;
    } else {
        // Preserve the Node information
        m_origin->RigidBodyDynamics::Math::Vector3d::operator=(position);
    }
}
const biorbd::utils::Vector3d& biorbd::muscles::Geometry::originInLocal() const
{
    return *m_origin;
}

void biorbd::muscles::Geometry::setInsertionInLocal(
    const utils::Vector3d &position)
{
    if (dynamic_cast<const biorbd::rigidbody::NodeSegment*>(&position)) {
        *m_insertion = position;
    } else {
        // Preserve the Node information
        m_insertion->RigidBodyDynamics::Math::Vector3d::operator=(position);
    }
}
const biorbd::utils::Vector3d &biorbd::muscles::Geometry::insertionInLocal()
const
{
    return *m_insertion;
}

// Position of the muscles in space
const biorbd::utils::Vector3d &biorbd::muscles::Geometry::originInGlobal() const
{
    biorbd::utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed at least once before calling originInLocal()");
    return *m_originInGlobal;
}
const biorbd::utils::Vector3d &biorbd::muscles::Geometry::insertionInGlobal()
const
{
    biorbd::utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed at least once before calling insertionInGlobal()");
    return *m_insertionInGlobal;
}
const std::vector<biorbd::utils::Vector3d>
&biorbd::muscles::Geometry::musclesPointsInGlobal() const
{
    biorbd::utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed at least once before calling musclesPointsInGlobal()");
    return *m_pointsInGlobal;
}

// Return the length and muscular velocity
const biorbd::utils::Scalar& biorbd::muscles::Geometry::length() const
{
    biorbd::utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed at least before calling length()");
    return *m_length;
}
const biorbd::utils::Scalar& biorbd::muscles::Geometry::musculoTendonLength()
const
{
    biorbd::utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed at least before calling length()");
    return *m_muscleTendonLength;
}
const biorbd::utils::Scalar& biorbd::muscles::Geometry::velocity() const
{
    biorbd::utils::Error::check(*m_isVelocityComputed,
                                "Geometry must be computed before calling velocity()");
    return *m_velocity;
}

// Return the Jacobian
const biorbd::utils::Matrix& biorbd::muscles::Geometry::jacobian() const
{
    biorbd::utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed before calling jacobian()");
    return *m_jacobian;
} // Return the last Jacobian
biorbd::utils::Matrix biorbd::muscles::Geometry::jacobianOrigin() const
{
    biorbd::utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed before calling jacobianOrigin()");
    return m_jacobian->block(0,0,3,m_jacobian->cols());
}
biorbd::utils::Matrix biorbd::muscles::Geometry::jacobianInsertion() const
{
    biorbd::utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed before calling jacobianInsertion()");
    return m_jacobian->block(m_jacobian->rows()-3,0,3,m_jacobian->cols());
}
biorbd::utils::Matrix biorbd::muscles::Geometry::jacobian(
    unsigned int idxViaPoint) const
{
    biorbd::utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed before calling jacobian(i)");
    return m_jacobian->block(3*idxViaPoint,0,3,m_jacobian->cols());
}

const biorbd::utils::Matrix &biorbd::muscles::Geometry::jacobianLength() const
{
    biorbd::utils::Error::check(*m_isGeometryComputed,
                                "Geometry must be computed before calling jacobianLength()");
    return *m_jacobianLength;
}

// --------------------------------------- //

void biorbd::muscles::Geometry::_updateKinematics(
    const biorbd::rigidbody::GeneralizedVelocity* Qdot,
    const biorbd::muscles::Characteristics* characteristics,
    biorbd::muscles::PathModifiers *pathModifiers)
{
    // Compute the length and velocities
    length(characteristics, pathModifiers);
    *m_isGeometryComputed = true;

    // Compute the jacobian of the lengths
    computeJacobianLength();
    if (Qdot != nullptr) {
        velocity(*Qdot);
        *m_isVelocityComputed = true;
    } else {
        *m_isVelocityComputed = false;
    }
}

const biorbd::utils::Vector3d &biorbd::muscles::Geometry::originInGlobal(
    biorbd::rigidbody::Joints &model,
    const biorbd::rigidbody::GeneralizedCoordinates &Q)
{
    // Return the position of the marker in function of the given position
    m_originInGlobal->block(0,0,3,
                            1) = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q,
                                    model.GetBodyId(m_origin->parent().c_str()), *m_origin,false);
    return *m_originInGlobal;
}

const biorbd::utils::Vector3d &biorbd::muscles::Geometry::insertionInGlobal(
    biorbd::rigidbody::Joints &model,
    const biorbd::rigidbody::GeneralizedCoordinates &Q)
{
    // Return the position of the marker in function of the given position
    m_insertionInGlobal->block(0,0,3,
                               1) = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, Q,
                                       model.GetBodyId(m_insertion->parent().c_str()), *m_insertion,false);
    return *m_insertionInGlobal;
}

void biorbd::muscles::Geometry::setMusclesPointsInGlobal(
    std::vector<biorbd::utils::Vector3d> &ptsInGlobal)
{
    biorbd::utils::Error::check(ptsInGlobal.size() >= 2,
                                "ptsInGlobal must at least have an origin and an insertion");
    m_pointsInLocal->clear(); // In this mode, we don't need the local, because the Jacobian of the points has to be given as well
    *m_pointsInGlobal = ptsInGlobal;
}

void biorbd::muscles::Geometry::setMusclesPointsInGlobal(
    biorbd::rigidbody::Joints &model,
    const biorbd::rigidbody::GeneralizedCoordinates &Q,
    biorbd::muscles::PathModifiers *pathModifiers)
{
    // Output varible (reset to zero)
    m_pointsInLocal->clear();
    m_pointsInGlobal->clear();

    // Do not apply on wrapping objects
    if (pathModifiers->nbWraps()!=0) {
        // CHECK TO MODIFY BEFOR GOING FORWARD WITH PROJECTS
        biorbd::utils::Error::check(pathModifiers->nbVia() == 0,
                                    "Cannot mix wrapping and via points yet") ;
        biorbd::utils::Error::check(pathModifiers->nbWraps() < 2,
                                    "Cannot compute more than one wrapping yet");

        // Get the matrix of Rt of the wrap
        biorbd::muscles::WrappingObject& w =
            static_cast<biorbd::muscles::WrappingObject&>(pathModifiers->object(0));
        const biorbd::utils::RotoTrans& RT = w.RT(model,Q);

        // Alias
        const biorbd::utils::Vector3d& po_mus = originInGlobal(model,
                                                Q);  // Origin on bone
        const biorbd::utils::Vector3d& pi_mus = insertionInGlobal(model,
                                                Q); // Insertion on bone

        biorbd::utils::Vector3d pi_wrap(0, 0,
                                        0); // point on the wrapping related to insertion
        biorbd::utils::Vector3d po_wrap(0, 0,
                                        0); // point on the wrapping related to origin

        biorbd::utils::Scalar a; // Force the computation of the length
        w.wrapPoints(RT,po_mus,pi_mus,po_wrap, pi_wrap, &a);

        // Store the points in local
        m_pointsInLocal->push_back(originInLocal());
        m_pointsInLocal->push_back(
            biorbd::utils::Vector3d(RigidBodyDynamics::CalcBodyToBaseCoordinates(
                                        model, Q, model.GetBodyId(w.parent().c_str()),po_wrap, false),
                                    "wrap_o", w.parent()));
        m_pointsInLocal->push_back(
            biorbd::utils::Vector3d(RigidBodyDynamics::CalcBodyToBaseCoordinates(
                                        model, Q, model.GetBodyId(w.parent().c_str()),pi_wrap, false),
                                    "wrap_i", w.parent()));
        m_pointsInLocal->push_back(insertionInLocal());

        // Store the points in global
        m_pointsInGlobal->push_back(po_mus);
        m_pointsInGlobal->push_back(po_wrap);
        m_pointsInGlobal->push_back(pi_wrap);
        m_pointsInGlobal->push_back(pi_mus);

    }

    else if (pathModifiers->nbObjects()!=0
             && pathModifiers->object(0).typeOfNode() ==
             biorbd::utils::NODE_TYPE::VIA_POINT) {
        m_pointsInLocal->push_back(originInLocal());
        m_pointsInGlobal->push_back(originInGlobal(model, Q));
        for (unsigned int i=0; i<pathModifiers->nbObjects(); ++i) {
            const biorbd::muscles::ViaPoint& node(static_cast<biorbd::muscles::ViaPoint&>
                                                  (pathModifiers->object(i)));
            m_pointsInLocal->push_back(node);
            m_pointsInGlobal->push_back(RigidBodyDynamics::CalcBodyToBaseCoordinates(model,
                                        Q,
                                        model.GetBodyId(node.parent().c_str()), node, false));
        }
        m_pointsInLocal->push_back(insertionInLocal());
        m_pointsInGlobal->push_back(insertionInGlobal(model,Q));

    } else if (pathModifiers->nbObjects()==0) {
        m_pointsInLocal->push_back(originInLocal());
        m_pointsInLocal->push_back(insertionInLocal());
        m_pointsInGlobal->push_back(originInGlobal(model, Q));
        m_pointsInGlobal->push_back(insertionInGlobal(model,Q));
    } else {
        biorbd::utils::Error::raise("Length for this type of object was not implemented");
    }

    // Set the dimension of jacobian
    setJacobianDimension(model);
}

const biorbd::utils::Scalar& biorbd::muscles::Geometry::length(
    const biorbd::muscles::Characteristics *characteristics,
    biorbd::muscles::PathModifiers *pathModifiers)
{
    *m_muscleTendonLength = 0;

    // because we can't combine, test the first (0) will let us know all the types if more than one
    if (pathModifiers != nullptr && pathModifiers->nbWraps()!=0) {
        biorbd::utils::Error::check(pathModifiers->nbVia() == 0,
                                    "Cannot mix wrapping and via points yet" ) ;
        biorbd::utils::Error::check(pathModifiers->nbWraps() < 2,
                                    "Cannot compute more than one wrapping yet");

        biorbd::utils::Vector3d pi_wrap(0, 0,
                                        0); // point on the wrapping related to insertion
        biorbd::utils::Vector3d po_wrap(0, 0,
                                        0); // point on the wrapping related to origin
        biorbd::utils::Scalar lengthWrap(0);
        static_cast<biorbd::muscles::WrappingObject&>(
            pathModifiers->object(0)).wrapPoints(po_wrap, pi_wrap, &lengthWrap);
        *m_muscleTendonLength = ((*m_pointsInGlobal)[0] - po_wrap).norm()
                                + // length before the wrap
                                lengthWrap                 + // length on the wrap
                                ((*m_pointsInGlobal)[3] - pi_wrap).norm();   // length after the wrap

    } else {
        for (unsigned int i=0; i<m_pointsInGlobal->size()-1; ++i) {
            *m_muscleTendonLength += ((*m_pointsInGlobal)[i+1] -
                                      (*m_pointsInGlobal)[i]).norm();
        }
    }

    *m_length = (*m_muscleTendonLength - characteristics->tendonSlackLength())
                /
                std::cos(characteristics->pennationAngle());

    return *m_length;
}

const biorbd::utils::Scalar& biorbd::muscles::Geometry::velocity(
    const biorbd::rigidbody::GeneralizedVelocity &Qdot)
{
    // Compute the velocity of the muscular elongation
    *m_velocity = (jacobianLength()*Qdot)[0];
    return *m_velocity;
}

void biorbd::muscles::Geometry::setJacobianDimension(biorbd::rigidbody::Joints
        &model)
{
    *m_jacobian = biorbd::utils::Matrix::Zero(static_cast<unsigned int>
                  (m_pointsInLocal->size()*3), model.dof_count);
    *m_G = biorbd::utils::Matrix::Zero(3, model.dof_count);
}

void biorbd::muscles::Geometry::jacobian(const biorbd::utils::Matrix &jaco)
{
    biorbd::utils::Error::check(jaco.rows()/3 == static_cast<int>
                                (m_pointsInGlobal->size()), "Jacobian is the wrong size");
    *m_jacobian = jaco;
}

void biorbd::muscles::Geometry::jacobian(
    biorbd::rigidbody::Joints &model,
    const biorbd::rigidbody::GeneralizedCoordinates &Q)
{
    for (unsigned int i=0; i<m_pointsInLocal->size(); ++i) {
        m_G->setZero();
        RigidBodyDynamics::CalcPointJacobian(model, Q,
                                             model.GetBodyId((*m_pointsInLocal)[i].parent().c_str()),
                                             (*m_pointsInLocal)[i], *m_G, false); // False for speed
        m_jacobian->block(3*i,0,3,model.dof_count) = *m_G;
    }
}

void biorbd::muscles::Geometry::computeJacobianLength()
{
    *m_jacobianLength = biorbd::utils::Matrix::Zero(1, m_jacobian->cols());

    // jacobian approximates as if there were no wrapping object
    const std::vector<biorbd::utils::Vector3d>& p = *m_pointsInGlobal;
    for (unsigned int i=0; i<p.size()-1 ; ++i) {
        *m_jacobianLength += (( p[i+1] - p[i] ).transpose() * (jacobian(i+1) - jacobian(
                                  i)))
                             /
                             ( p[i+1] - p[i] ).norm();
    }
}
