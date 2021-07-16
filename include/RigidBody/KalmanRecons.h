#ifndef BIORBD_RIGIDBODY_KALMAN_RECONS_H
#define BIORBD_RIGIDBODY_KALMAN_RECONS_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"


namespace biorbd
{
class Model;

namespace utils
{
class Matrix;
class Vector;
}

namespace rigidbody
{
class GeneralizedCoordinates;
class GeneralizedVelocity;
class GeneralizedAcceleration;

///
/// \brief Parameters of the reconstruction
///
class KalmanParam
{
public:
    ///
    /// \brief Set the Kalman filter parameters
    /// \param frequency The acquisition frequency express in Hertz
    /// \param noiseFactor The noise factor (on measurement matrix)
    /// \param errorFactor The error factor (on prediction matrix
    ///
    KalmanParam(
        double frequency = 100,
        double noiseFactor = 1e-10,
        double errorFactor = 1e-5);

    ///
    /// \brief Return the acquisition frequency
    ///
    double acquisitionFrequency() const;

    ///
    /// \brief Return the noise factor
    ///
    double noiseFactor() const;

    ///
    /// \brief Return the error factor
    ///
    double errorFactor() const;

private:
    double m_acquisitionFrequency; ///< The acquisition frequency
    double m_noiseFactor; ///< The noise factor
    double m_errorFactor; ///< The error factor
};

///
/// \brief Class Kinematic reconstruction algorithm using an Extended Kalman Filter
///
class BIORBD_API KalmanRecons
{
public:

    // Constructor

    ///
    /// \brief Kalman reconstruction
    ///
    KalmanRecons();

    ///
    /// \brief Kalman reconstruction
    /// \param model The joint model
    /// \param nbMeasure The number of measurements
    /// \param params The Kalman filter parameters
    ///
    KalmanRecons(
        biorbd::Model& model,
        unsigned int nbMeasure,
        KalmanParam params = KalmanParam());

    ///
    /// \brief Destroy class properly
    ///
    virtual ~KalmanRecons();

    ///
    /// \brief Deep copy of Kalman reconstruction
    /// \param other The Kalman reconstruction to copy
    ///
    void DeepCopy(const biorbd::rigidbody::KalmanRecons& other);


    ///
    /// \brief Get the state (Q, Qdot, Qddot)
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    ///
    void getState(
        biorbd::rigidbody::GeneralizedCoordinates *Q = nullptr,
        biorbd::rigidbody::GeneralizedVelocity *Qdot = nullptr,
        biorbd::rigidbody::GeneralizedAcceleration *Qddot = nullptr);

    ///
    /// \brief Set the initial guess of the reconstruction
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param Qddot The generalized accelerations
    ///
    void setInitState(
        const biorbd::rigidbody::GeneralizedCoordinates *Q = nullptr,
        const biorbd::rigidbody::GeneralizedVelocity *Qdot = nullptr,
        const biorbd::rigidbody::GeneralizedAcceleration *Qddot = nullptr);

    ///
    /// \brief Proceed to one iteration of the Kalman filter
    ///
    virtual void reconstructFrame() = 0;

protected:
    ///
    /// \brief Initialization of the filter
    ///
    virtual void initialize();

    ///
    /// \brief Create the evolution matrix
    /// \param m The number of degrees of freedom
    /// \param n The order of the Taylor development
    /// \param Te Is equal to \f$\frac{1}{\text{Acquisition frequency}}\f$
    /// \return The evolution matrix assuming constant frame rate
    ///
    biorbd::utils::Matrix evolutionMatrix(
        const unsigned int m,
        unsigned int n,
        double Te);

    ///
    /// \brief Process the noise matrix
    /// \param nbQ The number of degrees-of-freedom
    /// \param Te Is equal to \f$\frac{1}{\text{Acquisition frequency}}\f$
    /// \return The noise matrix
    ///
    biorbd::utils::Matrix processNoiseMatrix(
        const unsigned int nbQ,
        double Te);

    ///
    /// \brief Matrix of the noise on the measurements
    /// \param nbT The number of measurements
    /// \param val The noise level
    /// \return The matrix of the noise on the measurements
    ///
    biorbd::utils::Matrix measurementNoiseMatrix(
        const unsigned int nbT,
        double val);

    ///
    /// \brief Returns a initialized covianriance matrix
    /// \param nbQ The number of degrees-of-freedom
    /// \param val The initial value to fill the matrix with
    /// \return The initial covariance matrix
    ///
    biorbd::utils::Matrix initCovariance(
        const unsigned int nbQ,
        double val);

    ///
    /// \brief Initialize the states
    /// \param nbQ The number of degrees-of-freedom
    /// \return The initialized states
    ///
    biorbd::rigidbody::GeneralizedCoordinates initState(
        const unsigned int nbQ);

    ///
    /// \brief Compute an iteration of the Kalman filter
    /// \param measure The vector actual measurement to track
    /// \param projectedMeasure The projected measurement from the update step of the filter
    /// \param Hessian The hessian matrix
    /// \param occlusion The vector where occlusionsoccurs
    ///
    void iteration(
        biorbd::utils::Vector measure,
        const biorbd::utils::Vector &projectedMeasure,
        const biorbd::utils::Matrix &Hessian,
        const std::vector<unsigned int> &occlusion = std::vector<unsigned int>());

    ///
    /// \brief Manage the occlusion during the iteration
    /// \param InvTp The inverse of the Tp matrix
    /// \param measure The vector actual measurement to track
    /// \param occlusion The vector where occlusions occurs
    ///
    virtual void manageOcclusionDuringIteration(
        biorbd::utils::Matrix &InvTp,
        biorbd::utils::Vector &measure,
        const std::vector<unsigned int> &occlusion);

    // Variables attributes
    std::shared_ptr<KalmanParam> m_params; ///< The parameters of the Kalman filter
    std::shared_ptr<double> m_Te; ///< Inherent parameter to the frequency
    std::shared_ptr<unsigned int> m_nbDof; ///< Number of states
    std::shared_ptr<unsigned int> m_nMeasure; ///< Number of measurements

    // Kalman filter attributes
    std::shared_ptr<biorbd::utils::Vector> m_xp; ///< State vector
    std::shared_ptr<biorbd::utils::Matrix> m_A; ///< Evolution matrix
    std::shared_ptr<biorbd::utils::Matrix> m_Q; ///< Noise matrix
    std::shared_ptr<biorbd::utils::Matrix>
    m_R; ///< Matrix of the noise on the measurements
    std::shared_ptr<biorbd::utils::Matrix> m_Pp; ///< Covariance matrix

};

}
}

#endif // BIORBD_RIGIDBODY_KALMAN_RECONS_H
