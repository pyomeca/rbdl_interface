#define BIORBD_API_EXPORTS
#include "RigidBody/KalmanReconsIMU.h"

#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "BiorbdModel.h"
#include "Utils/Error.h"
#include "Utils/Matrix.h"
#include "Utils/Rotation.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/IMU.h"

biorbd::rigidbody::KalmanReconsIMU::KalmanReconsIMU() :
    biorbd::rigidbody::KalmanRecons(),
    m_PpInitial(std::make_shared<biorbd::utils::Matrix>()),
    m_firstIteration(std::make_shared<bool>(true))
{

}

biorbd::rigidbody::KalmanReconsIMU::KalmanReconsIMU(
    biorbd::Model &model,
    biorbd::rigidbody::KalmanParam params) :
    biorbd::rigidbody::KalmanRecons(model, model.nbTechIMUs()*9, params),
    m_PpInitial(std::make_shared<biorbd::utils::Matrix>()),
    m_firstIteration(std::make_shared<bool>(true))
{
    // Initialize the filter
    initialize();
}

biorbd::rigidbody::KalmanReconsIMU
biorbd::rigidbody::KalmanReconsIMU::DeepCopy() const
{
    biorbd::rigidbody::KalmanReconsIMU copy;
    copy.DeepCopy(*this);
    return copy;
}

void biorbd::rigidbody::KalmanReconsIMU::DeepCopy(const
        biorbd::rigidbody::KalmanReconsIMU &other)
{
    biorbd::rigidbody::KalmanRecons::DeepCopy(other);
    *m_PpInitial = *other.m_PpInitial;
    *m_firstIteration = *other.m_firstIteration;
}

void biorbd::rigidbody::KalmanReconsIMU::initialize()
{
    biorbd::rigidbody::KalmanRecons::initialize();

    // Keep in mind the m_Pp from the start
    *m_PpInitial = *m_Pp;
}

void biorbd::rigidbody::KalmanReconsIMU::manageOcclusionDuringIteration(
    biorbd::utils::Matrix &InvTp,
    biorbd::utils::Vector &measure,
    const std::vector<unsigned int> &occlusion)
{
    for (unsigned int i = 0; i < occlusion.size(); ++i)
        for (unsigned int j=occlusion[i] * 9; j< occlusion[i] * 9+9; ++j) {
            InvTp(j,j) =
                0; // Artefact due to the fact that m_R has a value at (j:j+2,j:j+2)
            measure[j] = 0;
        }
}

bool biorbd::rigidbody::KalmanReconsIMU::first()
{
    return *m_firstIteration;
}

void biorbd::rigidbody::KalmanReconsIMU::reconstructFrame(
    biorbd::Model &m,
    const std::vector<biorbd::rigidbody::IMU> &IMUobs,
    biorbd::rigidbody::GeneralizedCoordinates *Q,
    biorbd::rigidbody::GeneralizedVelocity *Qdot,
    biorbd::rigidbody::GeneralizedAcceleration *Qddot)
{
    // Separate the IMUobs in a big vector
    biorbd::utils::Vector T(static_cast<unsigned int>
                            (3*3*IMUobs.size())); // Matrix 3*3 * nbIMU
    for (unsigned int i=0; i<IMUobs.size(); ++i)
        for (unsigned int j=0; j<3; ++j) {
            T.block(9*i+3*j, 0, 3, 1) = IMUobs[i].block(0,j,3,1);
        }

    // Reconstruct the kinematics
    reconstructFrame(m, T, Q, Qdot, Qddot);
}


void biorbd::rigidbody::KalmanReconsIMU::reconstructFrame(
    biorbd::Model &model,
    const biorbd::utils::Vector &IMUobs,
    biorbd::rigidbody::GeneralizedCoordinates *Q,
    biorbd::rigidbody::GeneralizedVelocity *Qdot,
    biorbd::rigidbody::GeneralizedAcceleration *Qddot)
{
    // An iteration of the Kalman filter
    if (*m_firstIteration) {
        *m_firstIteration = false;
        for (unsigned int i=0; i<300; ++i) {
            // The first time, call in a recursive manner to have a decent initial position
            reconstructFrame(model, IMUobs, nullptr, nullptr, nullptr);

            // we don't need the velocity to get to the initial position
            // Otherwise, there are risks of overshooting
            m_xp->block(*m_nbDof, 0, *m_nbDof*2,
                        1) = biorbd::utils::Vector::Zero(*m_nbDof*2);
        }
    }

    // Projected state
    const biorbd::utils::Vector& xkm(*m_A * *m_xp);
    biorbd::rigidbody::GeneralizedCoordinates Q_tp(xkm.topRows(*m_nbDof));
    model.UpdateKinematicsCustom (&Q_tp, nullptr, nullptr);

    // Projected markers
    const std::vector<biorbd::rigidbody::IMU>& zest_tp = model.technicalIMU(Q_tp,
            false);
    // Jacobian
    const std::vector<biorbd::utils::Matrix>& J_tp = model.TechnicalIMUJacobian(
                Q_tp, false);
    // Create only one matrix for zest and Jacobian
    biorbd::utils::Matrix H(biorbd::utils::Matrix::Zero(*m_nMeasure,
                            *m_nbDof*3)); // 3*nCentrales => X,Y,Z ; 3*nbDof => Q, Qdot, Qddot
    biorbd::utils::Vector zest(biorbd::utils::Vector::Zero(*m_nMeasure));
    std::vector<unsigned int> occlusionIdx;
    for (unsigned int i=0; i<*m_nMeasure/9; ++i) {
        biorbd::utils::Scalar sum = 0;
        for (unsigned int j = 0; j < 9;
                ++j) { // Calculate the norm for the 9 components
            sum += IMUobs(i*9+j)*IMUobs(i*9+j);
        }
#ifdef BIORBD_USE_CASADI_MATH
        if (true) { // If there is an IMU (no zero or NaN)
#else
        if (sum != 0.0 && !std::isnan(sum)) { // If there is an IMU (no zero or NaN)
#endif
            H.block(i*9,0,9,*m_nbDof) = J_tp[i];
            const biorbd::utils::Rotation& rot = zest_tp[i].rot();
            for (unsigned int j = 0; j < 3; ++j) {
                zest.block(i*9+j*3, 0, 3, 1) = rot.block(0, j, 3, 1);
            }
        } else {
            occlusionIdx.push_back(i);
        }
    }

    // Make the filter
    iteration(IMUobs, zest, H, occlusionIdx);

    getState(Q, Qdot, Qddot);
}

void biorbd::rigidbody::KalmanReconsIMU::reconstructFrame()
{
    biorbd::utils::Error::raise("Reconstructing kinematics for IMU needs measurements");
}
