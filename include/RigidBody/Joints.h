#ifndef BIORBD_RIGIDBODY_JOINTS_H
#define BIORBD_RIGIDBODY_JOINTS_H

#include <memory>
#include <rbdl/Model.h>
#include <rbdl/Constraints.h>
#include "biorbdConfig.h"
#include "Utils/Scalar.h"

namespace biorbd
{
namespace utils
{
class String;
class RotoTrans;
class Matrix;
class Vector;
class Vector3d;
class Range;
class SpatialVector;
}

namespace rigidbody
{
class GeneralizedCoordinates;
class GeneralizedVelocity;
class GeneralizedAcceleration;
class GeneralizedTorque;
class NodeSegment;
class MeshFace;
class Segment;
class SegmentCharacteristics;
class Mesh;
class Contacts;

///
/// \brief This is the core of the musculoskeletal model in biorbd
///
#ifdef SWIG
class BIORBD_API Joints
#else
class BIORBD_API Joints : public RigidBodyDynamics::Model
#endif
{
public:

    ///
    /// \brief Construct a joint model
    ///
    Joints();

    ///
    /// \brief Construct a joint model from another model
    /// \param other The other joint model
    ///
    Joints(
        const biorbd::rigidbody::Joints& other);

    ///
    /// \brief Properly destroy class
    ///
    virtual ~Joints();

    ///
    /// \brief Deep copy of the joints
    /// \return Copy of the joints
    ///
    biorbd::rigidbody::Joints DeepCopy() const;

    ///
    /// \brief Deep copy of the joints
    /// \param other The joints to copy
    ///
    void DeepCopy(
        const biorbd::rigidbody::Joints& other);

    ///
    /// \brief Add a segment to the model
    /// \param segmentName Name of the segment
    /// \param parentName Name of the segment parent
    /// \param translationSequence The translation sequence
    /// \param rotationSequence Euler sequence of rotations
    /// \param QRanges Ranges of the translations and rotations dof. The length of QRanges must be equal to length of translations and rotations
    /// \param QDotRanges Ranges of the translations and rotations dof velocity. The length of QDotRanges must be equal to length of translations and rotations
    /// \param QDDotRanges Ranges of the translations and rotations dof acceleration. The length of QDDotRanges must be equal to length of translations and rotations
    /// \param characteristics The characteristics of the semgent (mass, center of mass, inertia of the segment, etc)
    /// \param referenceFrame Transformation of the parent to child
    /// \param forcePlates The number of the force platform attached to the Segment (if -1 no force platform is attached)
    ///
    unsigned int AddSegment(
        const biorbd::utils::String &segmentName,
        const biorbd::utils::String &parentName,
        const biorbd::utils::String &translationSequence,
        const biorbd::utils::String &rotationSequence,
        const std::vector<biorbd::utils::Range>& QRanges,
        const std::vector<biorbd::utils::Range>& QDotRanges,
        const std::vector<biorbd::utils::Range>& QDDotRanges,
        const biorbd::rigidbody::SegmentCharacteristics& characteristics,
        const biorbd::utils::RotoTrans& referenceFrame,
        int forcePlates=-1);

    ///
    /// \brief Add a segment to the model
    /// \param segmentName Name of the segment
    /// \param parentName Name of the segment parent
    /// \param translationSequence The translation sequence
    /// \param QRanges Ranges of the translations and rotations dof. The length of QRanges must be equal to length of translations and rotations
    /// \param QDotRanges Ranges of the translations and rotations dof velocity. The length of QDotRanges must be equal to length of translations and rotations
    /// \param QDDotRanges Ranges of the translations and rotations dof acceleration. The length of QDDotRanges must be equal to length of translations and rotations
    /// \param characteristics The characteristics of the semgent (mass, center of mass, inertia of the segment, etc)
    /// \param referenceFrame Transformation of the parent to child
    /// \param forcePlates The number of the force platform attached to the Segment (if -1 no force platform is attached)
    ///
    unsigned int AddSegment(
        const biorbd::utils::String &segmentName,
        const biorbd::utils::String &parentName,
        const biorbd::utils::String &translationSequence,
        const std::vector<biorbd::utils::Range>& QRanges,
        const std::vector<biorbd::utils::Range>& QDotRanges,
        const std::vector<biorbd::utils::Range>& QDDotRanges,
        const biorbd::rigidbody::SegmentCharacteristics& characteristics,
        const biorbd::utils::RotoTrans& referenceFrame,
        int forcePlates=-1);


    // -- GENERAL MODELLING -- //
    ///
    /// \brief Get the current gravity
    /// \return The current gravity
    ///
    biorbd::utils::Vector3d getGravity() const;

    ///
    /// \brief Set the gravity
    /// \param newGravity The new gravity vector
    ///
    void setGravity(
        const biorbd::utils::Vector3d& newGravity);

    // -- INFORMATION ON THE MODEL -- //
    ///
    /// \brief Return the biorbd body identification
    /// \param segmentName The name of the segment
    /// \return The biorbd body identification
    ///
    int GetBodyBiorbdId(
        const biorbd::utils::String &segmentName) const;

    ///
    /// \brief Return the number of generalized torque
    /// \return The number of generalized torque
    ///
    unsigned int nbGeneralizedTorque() const;

    ///
    /// \brief Return the actual number of segment
    /// \return The actual number of segment
    ///
    unsigned int nbSegment() const;

    ///
    /// \brief Return the number of degrees of freedom (DoF)
    /// \return The number of DoF
    ///
    unsigned int nbDof() const;

    ///
    /// \brief Return the index of a DoF in a segment
    /// \param SegmentName The name of the Segment
    /// \param dofName The name of the degree of freedom (DoF)
    /// \return The index of a DoF in a segment
    ///
    unsigned int getDofIndex(
        const biorbd::utils::String& SegmentName,
        const biorbd::utils::String& dofName);

    ///
    /// \brief Return the names of the degree of freedom (DoF)
    /// \return The names of the DoF
    ///
    std::vector<biorbd::utils::String> nameDof() const;

    ///
    /// \brief Return the number of generalized coordinates (Q)
    /// \return The number of Q
    ///
    unsigned int nbQ() const;

    ///
    /// \brief Return the number of generalized velocities (Qdot)
    /// \return The number of Qdot
    ///
    unsigned int nbQdot() const;

    ///
    /// \brief Return the number of generalized acceleration (Qddot)
    /// \return The number of Qddot
    ///
    unsigned int nbQddot() const;

    ///
    /// \brief Return the dof on the root
    /// \return The dof on the root
    ///
    unsigned int nbRoot() const;

    ///
    /// \brief Return the number of segments that are described using quaternions
    /// \return The number number of segments
    ///
    unsigned int nbQuat() const;


    ///
    /// \brief updateSegmentCharacteristics Change the inertia characteristics of the segment idx
    /// \param idx The index of the segment to change
    /// \param characteristics The new characteristics
    ///
    /// Warning: This function may behave surpringly due to the core of RBDL. The
    /// new characteristic values will replace everything which is attach in a fixed manner
    /// (that is no degrees-of-freedom). So if your model has 3 segments, but only the first
    /// one has dof (and the rest is rigidly attached to the first), then it doesn't matter
    /// if idx is 0, 1 or 2, because RBDL considers that all these segment are 1 segment.
    /// It is therefore expected that characteristics is the combination of mass and
    /// inertia for these 3 segments as well.
    ///
    void updateSegmentCharacteristics(
        unsigned int idx,
        const biorbd::rigidbody::SegmentCharacteristics& characteristics);


    ///
    /// \brief Get a segment of index idx
    /// \param idx Index of the segment
    /// \return The segment
    ///
    const biorbd::rigidbody::Segment& segment(
        unsigned int idx) const;

    ///
    /// \brief Get a segment of a specific name
    /// \param name The name of the segment to return
    /// \return The segment
    ///
    const biorbd::rigidbody::Segment& segment(
        const biorbd::utils::String& name) const;
    // ------------------------------ //


    // -- FORCE PLATE DISPATCHER -- //
    ///
    /// \brief Dispatch the forces from the force plate in a vector
    /// \param spatialVector The values over time of one spatial vector per force platform
    /// \param frame The frame to dispatch
    /// \return A spatial vector with the forces
    ///
    std::vector<RigidBodyDynamics::Math::SpatialVector> dispatchedForce(
        std::vector<std::vector<biorbd::utils::SpatialVector>> &spatialVector,
        unsigned int frame) const;

    ///
    /// \brief Dispatch the forces from the force plate in a spatial vector
    /// \param sv One spatial vector per force platform
    /// \return A spatial vector with the forces
    ///
    std::vector<RigidBodyDynamics::Math::SpatialVector> dispatchedForce(
        std::vector<biorbd::utils::SpatialVector> &sv) const;
    // ---------------------------- //


    ///
    /// \brief Update the kinematic variables such as body velocities and accelerations in the model to reflect the variables passed to this function
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    ///
    void UpdateKinematicsCustom(
        const biorbd::rigidbody::GeneralizedCoordinates *Q = nullptr,
        const biorbd::rigidbody::GeneralizedVelocity *Qdot = nullptr,
        const biorbd::rigidbody::GeneralizedAcceleration *Qddot = nullptr);


    // -- POSITION INTERFACE OF THE MODEL -- //

    ///
    /// \brief Return the joint coordinate system (JCS) in global reference frame at a given Q
    /// \param Q The generalized coordinates
    /// \return The JCS in global reference frame at a given Q
    ///
    std::vector<biorbd::utils::RotoTrans> allGlobalJCS(
        const biorbd::rigidbody::GeneralizedCoordinates &Q);

    ///
    /// \brief Return the joint coordinate system (JCS) in global reference frame at a given Q
    /// \return The JCS in global reference frame
    ///
    /// This function assumes kinematics has been already updated
    ///
    std::vector<biorbd::utils::RotoTrans> allGlobalJCS() const;

    ///
    /// \brief Return the joint coordinate system (JCS) for the segment in global reference frame at a given Q
    /// \param Q The generalized coordinates
    /// \param name The name of the segment
    /// \return The JCS of the segment in global reference frame at a given Q
    ///
    biorbd::utils::RotoTrans globalJCS(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::utils::String &name);

    ///
    /// \brief Return the joint coordinate system (JCS) for the segment idx in global reference frame at a given Q
    /// \param Q The generalized coordinates
    /// \param idx The index of the segment
    /// \return The JCS of the segment idx in global reference frame at a given Q
    ///
    biorbd::utils::RotoTrans globalJCS(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        unsigned int idx);

    ///
    /// \brief Return the joint coordinate system (JCS) for the segment in global reference
    /// \param name The name of the segment
    /// \return The JCS of the segment in global reference frame
    ///
    /// This function assumes kinematics has been already updated
    ///
    biorbd::utils::RotoTrans globalJCS(
        const biorbd::utils::String &name) const;

    ///
    /// \brief Return the joint coordinate system (JCS) for the segment idx in global reference
    /// \param idx The index of the segment
    /// \return The JCS of the segment idx in global reference frame
    ///
    /// This function assumes kinematics has been already updated
    ///
    biorbd::utils::RotoTrans globalJCS(
        unsigned int idx) const;

    ///
    /// \brief Return all the joint coordinate system (JCS) in its parent reference frame
    /// \return All the JCS in parent reference frame
    ///
    std::vector<biorbd::utils::RotoTrans> localJCS() const;

    ///
    /// \brief Return the joint coordinate system (JCS) of the segment its parent reference frame
    /// \param name The name of the segment
    /// \return The JCS of the segment in parent reference frame
    ///
    biorbd::utils::RotoTrans localJCS(
        const biorbd::utils::String &name) const;

    ///
    /// \brief Return the joint coordinate system (JCS) of the segment idx its parent reference frame
    /// \param idx The index of the segment
    /// \return The JCS of the segment idx in parent reference frame
    ///
    biorbd::utils::RotoTrans localJCS(
        const unsigned int idx) const;

    ///
    /// \brief Project a point on specific axis of a segment
    /// \param Q The generalized coordinates
    /// \param v The point to project
    /// \param segmentIdx The segment index to project the marker on
    /// \param axesToRemove The axis to remove
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The position of the projected marker
    ///
    biorbd::rigidbody::NodeSegment projectPoint(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::utils::Vector3d &v,
        int segmentIdx,
        const biorbd::utils::String& axesToRemove,
        bool updateKin=true);

    ///
    /// \brief Project multiples points on their respective segment
    /// \param Q The generalized coordinates
    /// \param v All the points to project. The number of points must match the number of marker in the joint model
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The projected markers from points corresponding to markers from the model
    ///
    /// Return projected markers from points corresponding to markers from the model.
    /// The vector needs to be equal to the number of markers and in the order given
    /// by Markers and in global coordinates
    ///
    std::vector<biorbd::rigidbody::NodeSegment>  projectPoint(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const std::vector<biorbd::rigidbody::NodeSegment> &v,
        bool updateKin=true);

    ///
    /// \brief Return the projected markers from a point corresponding to a marker from the model
    /// \param Q The generalized coordinates
    /// \param n A reference to a marker from the model
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The projected markers from a point corresponding to a marker from the model
    ///
    biorbd::rigidbody::NodeSegment projectPoint(
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::NodeSegment& n,
        bool updateKin);

    ///
    /// \brief Return the jacobian matrix of the projected markers for a marker from the model
    /// \param Q The generalized coordinates
    /// \param p A reference to a marker from the model
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The jacobian matrix of the projected marker
    ///
    biorbd::utils::Matrix projectPointJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        biorbd::rigidbody::NodeSegment p,
        bool updateKin);

    ///
    /// \brief Return the Jacobian matrix of a projected marker on the segment segmentIdx
    /// \param Q The generalized coordinates
    /// \param v The marker to project
    /// \param segmentIdx The index of the segment to project the marker on
    /// \param axesToRemove The axes to remove
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The Jacobian matrix of a projected marker on the segment segmentIdx
    ///
    biorbd::utils::Matrix projectPointJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::utils::Vector3d &v,
        int segmentIdx,
        const biorbd::utils::String& axesToRemove,
        bool updateKin);

    ///
    /// \brief Return the jacobian matrix of the projected markers
    /// \param Q The generalized coordinates
    /// \param v All the markers. The size of which must match the one in the joint model
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The jacobian matrix of the projected markers
    ///
    /// Return the jacobian matrix of projected markers from points corresponding to markers from the model.
    /// The vector needs to be equal to the number of markers and in the order given
    /// by Markers and in global coordinates
    ///
    std::vector<biorbd::utils::Matrix> projectPointJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const std::vector<biorbd::rigidbody::NodeSegment> &v,
        bool updateKin);
    // ------------------------------------- //


    // -- MASS RELATED STUFF -- //
    ///
    /// \brief Return the total mass of the model
    /// \return The toal mass of the model
    ///
    biorbd::utils::Scalar mass() const;

    ///
    /// \brief Return the position of the center of mass
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The position of the center of mass
    ///
    biorbd::utils::Vector3d CoM(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin=true);

    ///
    /// \brief Return the position of the center of mass of each segment
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The position of the center of mass of each segment
    ///
    std::vector<biorbd::rigidbody::NodeSegment> CoMbySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin=true);

    ///
    /// \brief Return the position of the center of mass of each segment in a matrix
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The position of the center of mass of each segment
    ///
    biorbd::utils::Matrix CoMbySegmentInMatrix(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin=true);

    ///
    /// \brief Return the position of the center of mass of segment idx
    /// \param Q The generalized coordinates
    /// \param idx The index of the segment
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The position of the center of mass of segment idx
    ///
    biorbd::utils::Vector3d CoMbySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const unsigned int idx,
        bool updateKin=true);

    ///
    /// \brief Return the velocity of the center of mass
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The velocity of the center of mass
    ///
    biorbd::utils::Vector3d CoMdot(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedVelocity &Qdot,
        bool updateKin=true);

    ///
    /// \brief Return the acceleration of the center of mass
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The acceleration variables
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The acceleration of the center of mass
    ///
    biorbd::utils::Vector3d CoMddot(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedVelocity &Qdot,
        const biorbd::rigidbody::GeneralizedAcceleration &Qddot,
        bool updateKin=true);

    ///
    /// \brief Return the velocity of the center of mass of each segment
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The velocity of the center of mass of each segment
    ///
    std::vector<biorbd::utils::Vector3d> CoMdotBySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedVelocity &Qdot,
        bool updateKin=true);

    ///
    /// \brief Return the velocity of the center of mass of segment idx
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param idx The index of the segment
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The velocity of the center of mass of segment idx
    ///
    biorbd::utils::Vector3d CoMdotBySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedVelocity &Qdot,
        const unsigned int idx,
        bool updateKin=true);

    ///
    /// \brief Return the acceleration of the center of mass of each segment
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The acceleration variables
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The acceleration of the center of mass of each segment
    ///
    std::vector<biorbd::utils::Vector3d> CoMddotBySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedVelocity &Qdot,
        const biorbd::rigidbody::GeneralizedAcceleration &Qddot,
        bool updateKin=true);

    ///
    /// \brief Return the acceleration of the center of mass of segment idx
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The acceleration variables
    /// \param idx The segment identification
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The acceleration of the center of mass of segment idx
    ///
    biorbd::utils::Vector3d CoMddotBySegment(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedVelocity &Qdot,
        const biorbd::rigidbody::GeneralizedAcceleration &Qddot,
        const unsigned int idx,
        bool updateKin = true);

    ///
    /// \brief Return the jacobian matrix of the center of mass
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The jacobian matrix of the center of mass
    ///
    biorbd::utils::Matrix CoMJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin = true);
    // ------------------------ //


    // -- MESH OF THE MODEL -- //
    ///
    /// \brief Return the vertices of the mesh for all segments in global reference frame
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The vertices of the for all segments
    ///
    std::vector<std::vector<biorbd::utils::Vector3d>> meshPoints(
                const biorbd::rigidbody::GeneralizedCoordinates &Q,
                bool updateKin = true);

    ///
    /// \brief Return the vertices of the mesh for the segment idx
    /// \param Q The generalized coordinates
    /// \param idx The index of the segment
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The vertices of the of the segment idx
    ///
    std::vector<biorbd::utils::Vector3d> meshPoints(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        unsigned int idx,
        bool updateKin = true);

    ///
    /// \brief Return all the vertices of the mesh points in a matrix
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics of the model should be computed
    /// \return All the vertices
    ///
    std::vector<biorbd::utils::Matrix> meshPointsInMatrix(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin = true
    );

    ///
    /// \brief Return the mesh faces for all the segments
    /// \return The mesh faces for all the segments
    ///
    std::vector<std::vector<MeshFace> > meshFaces() const;

    ///
    /// \brief Return the mesh faces for segment idx
    /// \param idx The index of the segment
    /// \return The mesh face for segment idx
    ///
    const std::vector<biorbd::rigidbody::MeshFace> &meshFaces(
        unsigned int idx) const;

    ///
    /// \brief Return the segment mesh
    /// \return The segment mesh
    ///
    std::vector<biorbd::rigidbody::Mesh> mesh() const;

    ///
    /// \brief Return the segment mesh for segment idx
    /// \param idx The index of the segment
    /// \return The Segment mesh for segment idx
    ///
    const biorbd::rigidbody::Mesh& mesh(
        unsigned int  idx) const;
    // ----------------------- //


    // -- ANGULAR MOMENTUM FUNCTIONS -- //
    ///
    /// \brief Calculate the angular momentum of the center of mass
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The angular momentum of the center of mass
    ///
    biorbd::utils::Vector3d angularMomentum(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedVelocity &Qdot,
        bool updateKin = true); // Wrapper pour le moment angulaire

    ///
    /// \brief Get the mass matrix at a given position Q
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics should be updated
    /// \return The mass matrix
    ///
    biorbd::utils::Matrix massMatrix(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin = true);

    ///
    /// \brief Calculate the angular momentum of the center of mass
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The angular momentum of the center of mass
    ///
    biorbd::utils::Vector3d CalcAngularMomentum (
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedVelocity &Qdot,
        bool updateKin);

    ///
    /// \brief Calculate the angular momentum of the center of mass
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The angular momentum of the center of mass
    ///
    biorbd::utils::Vector3d CalcAngularMomentum (
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedVelocity &Qdot,
        const biorbd::rigidbody::GeneralizedAcceleration &Qddot,
        bool updateKin);

    ///
    /// \brief Calculate the segment center of mass angular momentum
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The segments center of mass angular momentum
    ///
    std::vector<biorbd::utils::Vector3d> CalcSegmentsAngularMomentum (
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedVelocity &Qdot,
        bool updateKin);

    ///
    /// \brief Calculate the segment center of mass angular momentum
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The segments center of mass angular momentum
    ///
    std::vector<biorbd::utils::Vector3d> CalcSegmentsAngularMomentum (
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedVelocity &Qdot,
        const biorbd::rigidbody::GeneralizedAcceleration &Qddot,
        bool updateKin);
    // -------------------------------- //

    ///
    /// \brief Calculate the jacobian matrix of a rotation matrix
    /// \param Q The generalized coordinates
    /// \param segmentIdx The index of the segment
    /// \param rotation The rotation matrix
    /// \param G The jacobian matrix (output)
    /// \param updateKin If the kinematics of the model should be computed
    ///
    void CalcMatRotJacobian (
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        unsigned int segmentIdx,
        const RigidBodyDynamics::Math::Matrix3d &rotation,
        RigidBodyDynamics::Math::MatrixNd &G,
        bool updateKin);

    ///
    /// \brief Return the derivate of Q in function of Qdot (if not Quaternion, Qdot is directly returned)
    /// \param Q The generalized coordinates
    /// \param QDot The generalized velocities
    /// \param k_stab
    /// \return The derivate of Q in function of Qdot
    ///
    biorbd::rigidbody::GeneralizedVelocity computeQdot(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedCoordinates &QDot,
        const double k_stab = 1);

    // ---- DYNAMIC INTERFACE ---- //
    ///
    /// \brief Interface for the inverse dynamics of RBDL
    /// \param Q The Generalized Coordinates
    /// \param QDot The Generalized Velocities
    /// \param QDDot The Generalzed Acceleration
    /// \param f_ext External force acting on the system if there are any
    /// \return The Generalized Torques
    ///
    biorbd::rigidbody::GeneralizedTorque InverseDynamics(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedVelocity &QDot,
        const biorbd::rigidbody::GeneralizedAcceleration &QDDot,
        std::vector<biorbd::utils::SpatialVector>* f_ext = nullptr);

    ///
    /// \brief Interface to NonLinearEffect
    /// \param Q The Generalized Coordinates
    /// \param QDot The Generalized Velocities
    /// \param f_ext External force acting on the system if there are any
    /// \return The Generalized Torques of the bias effects
    ///
    biorbd::rigidbody::GeneralizedTorque NonLinearEffect(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const biorbd::rigidbody::GeneralizedVelocity &QDot,
        std::vector<biorbd::utils::SpatialVector>* f_ext = nullptr);

    ///
    /// \brief Interface for the forward dynamics of RBDL
    /// \param Q The Generalized Coordinates
    /// \param QDot The Generalized Velocities
    /// \param Tau The Generalized Torques
    /// \param f_ext External force acting on the system if there are any
    /// \return The Generalized Accelerations
    ///
    biorbd::rigidbody::GeneralizedAcceleration ForwardDynamics(
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedVelocity& QDot,
        const biorbd::rigidbody::GeneralizedTorque& Tau,
        std::vector<biorbd::utils::SpatialVector>* f_ext = nullptr);

    ///
    /// \brief Interface for the forward dynamics with contact of RBDL
    /// \param Q The Generalized Coordinates
    /// \param QDot The Generalized Velocities
    /// \param Tau The Generalized Torques
    /// \param CS The Constraint set that will be filled
    /// \param f_ext External force acting on the system if there are any
    /// \return The Generalized Accelerations
    ///
    biorbd::rigidbody::GeneralizedAcceleration ForwardDynamicsConstraintsDirect(
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedVelocity& QDot,
        const biorbd::rigidbody::GeneralizedTorque& Tau,
        biorbd::rigidbody::Contacts& CS,
        std::vector<biorbd::utils::SpatialVector>* f_ext = nullptr);

    ///
    /// \brief Interface for contacts of the forward dynamics with contact of RBDL
    /// \param Q The Generalized Coordinates
    /// \param QDot The Generalized Velocities
    /// \param Tau The Generalized Torques
    /// \param f_ext External force acting on the system if there are any
    /// \return The Contraint set
    ///
    biorbd::utils::Vector ContactForcesFromForwardDynamicsConstraintsDirect(
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedVelocity& QDot,
        const biorbd::rigidbody::GeneralizedTorque& Tau,
        std::vector<biorbd::utils::SpatialVector>* f_ext = nullptr);

    ///
    /// \brief Interface for the forward dynamics with contact of RBDL
    /// \param Q The Generalized Coordinates
    /// \param QDot The Generalized Velocities
    /// \param Tau The Generalized Torques
    /// \param f_ext External force acting on the system if there are any
    /// \return The Generalized Accelerations
    ///
    biorbd::rigidbody::GeneralizedAcceleration ForwardDynamicsConstraintsDirect(
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedVelocity& QDot,
        const biorbd::rigidbody::GeneralizedTorque& Tau,
        std::vector<biorbd::utils::SpatialVector>* f_ext = nullptr);

    ///
    /// \brief Compute the QDot post from an impact
    /// \param Q The Generalized Coordinates
    /// \param QDotPre The Generalized Velocities before impact
    /// \return The Generalized Velocities post acceleration
    ///
    biorbd::rigidbody::GeneralizedVelocity ComputeConstraintImpulsesDirect(
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedVelocity& QDotPre);

protected:
    std::shared_ptr<std::vector<biorbd::rigidbody::Segment>>
            m_segments; ///< All the articulations

    std::shared_ptr<unsigned int>
    m_nbRoot; ///< The number of DoF on the root segment
    std::shared_ptr<unsigned int>
    m_nbDof; ///< The total number of degrees of freedom
    std::shared_ptr<unsigned int> m_nbQ; ///< The total number of Q
    std::shared_ptr<unsigned int> m_nbQdot; ///< The total number of Qdot
    std::shared_ptr<unsigned int> m_nbQddot; ///< The total number of Qddot
    std::shared_ptr<unsigned int>
    m_nRotAQuat; ///< The number of segments per quaternion
    std::shared_ptr<bool>
    m_isKinematicsComputed; ///< If the kinematics are computed
    std::shared_ptr<biorbd::utils::Scalar>
    m_totalMass; ///< Mass of all the bodies combined

    ///
    /// \brief Calculate the joint coordinate system (JCS) in global reference frame of a specified segment
    /// \param Q The generalized coordinates
    /// \param segmentIdx The index of the segment
    /// \param updateKin If the kinematics of the model should be computed
    /// \return The JCS of the segment in global reference frame
    ///
    RigidBodyDynamics::Math::SpatialTransform CalcBodyWorldTransformation(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        const unsigned int segmentIdx,
        bool updateKin = true);

    ///
    /// \brief Calculate the joint coordinate system (JCS) in global of a specified segment
    /// \param segmentIdx The index of the segment
    /// \return The JCS in global
    ///
    /// This function assumes that the kinematics was previously updated
    ///
    RigidBodyDynamics::Math::SpatialTransform CalcBodyWorldTransformation(
        const unsigned int segmentIdx) const;

    ///
    /// \brief Return the mesh vertices of segment idx
    /// \param RT The RotoTrans of the segment
    /// \param idx The index of the segment
    /// \return The mesh vertices attached to a segment idx
    ///
    std::vector<biorbd::utils::Vector3d> meshPoints(
        const std::vector<biorbd::utils::RotoTrans> &RT,
        unsigned int idx) const;

public:
    ///
    /// \brief Check for the Generalized coordinates, velocities, acceleration and torque dimensions
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    /// \param torque The generalized torques
    ///
    void checkGeneralizedDimensions(
        const biorbd::rigidbody::GeneralizedCoordinates *Q = nullptr,
        const biorbd::rigidbody::GeneralizedVelocity *Qdot = nullptr,
        const biorbd::rigidbody::GeneralizedAcceleration *Qddot = nullptr,
        const biorbd::rigidbody::GeneralizedTorque *torque = nullptr);
};

}
}

#endif // BIORBD_RIGIDBODY_JOINTS_H
