#ifndef BIORBD_RIGIDBODY_KALMAN_RECONS_MARKERS_HPP
#define BIORBD_RIGIDBODY_KALMAN_RECONS_MARKERS_HPP

#include "biorbdConfig.h"
#include "RigidBody/KalmanRecons.h"


namespace biorbd
{
namespace rigidbody
{
class Markers;
class NodeSegment;

///
/// \brief Class Kinematic reconstruction algorithm using an Extended Kalman Filter using skin markers
///
class BIORBD_API KalmanReconsMarkers : public biorbd::rigidbody::KalmanRecons
{
public:

    // Constructor

    ///
    /// \brief Initialize the Kalman filter and Kalman reconstruction for Markers data
    ///
    KalmanReconsMarkers();

    ///
    /// \brief Initialize the Kalman filter and Kalman reconstruction for Markers data
    /// \param model The joint model
    /// \param params The Kalman filter parameters
    ///
    KalmanReconsMarkers(
        biorbd::Model& model,
        biorbd::rigidbody::KalmanParam params = biorbd::rigidbody::KalmanParam());

    ///
    /// \brief Deep copy of the Kalman reconstruction
    /// \return Copy of the Kalman reconstruction
    ///
    biorbd::rigidbody::KalmanReconsMarkers DeepCopy() const;

    ///
    /// \brief Deep copy of the Kalman reconstruction
    /// \param other The Kalman reconstruction to copy
    ///
    void DeepCopy(const biorbd::rigidbody::KalmanReconsMarkers& other);

    ///
    /// \brief Reconstruct the kinematics from markers data
    /// \param model The joint model
    /// \param Tobs The observed markers
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    /// \param removeAxes If the algo should ignore or not the removeAxis defined in the bioMod file
    ///
    virtual void reconstructFrame(
        biorbd::Model &model,
        const biorbd::rigidbody::Markers &Tobs,
        biorbd::rigidbody::GeneralizedCoordinates *Q,
        biorbd::rigidbody::GeneralizedVelocity *Qdot,
        biorbd::rigidbody::GeneralizedAcceleration *Qddot,
        bool removeAxes=true);

    ///
    /// \brief Reconstruct the kinematics from markers data
    /// \param model The joint model
    /// \param Tobs The observed markers
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    /// \param removeAxes If the algo should ignore or not the removeAxis defined in the bioMod file
    ///
    virtual void reconstructFrame(
        biorbd::Model &model,
        const std::vector<biorbd::rigidbody::NodeSegment> &Tobs,
        biorbd::rigidbody::GeneralizedCoordinates *Q,
        biorbd::rigidbody::GeneralizedVelocity *Qdot,
        biorbd::rigidbody::GeneralizedAcceleration *Qddot,
        bool removeAxes=true);

    ///
    /// \brief Reconstruct the kinematics from markers data
    /// \param model The joint model
    /// \param Tobs The observed markers in a column-major vector
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    /// \param removeAxes If the algo should ignore or not the removeAxis defined in the bioMod file
    ///
    virtual void reconstructFrame(
        biorbd::Model &model,
        const biorbd::utils::Vector &Tobs,
        biorbd::rigidbody::GeneralizedCoordinates *Q = nullptr,
        biorbd::rigidbody::GeneralizedVelocity *Qdot = nullptr,
        biorbd::rigidbody::GeneralizedAcceleration *Qddot = nullptr,
        bool removeAxes=true);

    ///
    /// \brief This function cannot be used to reconstruct frames
    ///
    virtual void reconstructFrame();

    ///
    /// \brief Return if the first iteration was done
    /// \return If the first iteration was done
    ///
    bool first();

protected:
    ///
    /// \brief Initialization of the filter
    ///
    virtual void initialize();

    ///
    /// \brief Manage the occlusion during the iteration
    /// \param InvTp The inverse of the Tp matrix
    /// \param measure The vector actual measurement to track
    /// \param occlusion The vector where occlusions occurs
    ///
    virtual void manageOcclusionDuringIteration(
        biorbd::utils::Matrix& InvTp,
        biorbd::utils::Vector &measure,
        const std::vector<unsigned int> &occlusion);

    std::shared_ptr<biorbd::utils::Matrix>
    m_PpInitial; ///< Initial covariance matrix
    std::shared_ptr<bool> m_firstIteration; ///< If first iteration was done
};

}
}

#endif // BIORBD_RIGIDBODY_KALMAN_RECONS_MARKERS_HPP
