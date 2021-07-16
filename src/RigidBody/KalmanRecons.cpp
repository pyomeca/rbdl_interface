#define BIORBD_API_EXPORTS
#include "RigidBody/KalmanRecons.h"

#include "BiorbdModel.h"
#include "Utils/Matrix.h"
#include "Utils/Vector.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedAcceleration.h"

biorbd::rigidbody::KalmanRecons::KalmanRecons() :
    m_params(std::make_shared<KalmanParam>()),
    m_Te(std::make_shared<double>(1.0/(m_params->acquisitionFrequency()))),
    m_nbDof(std::make_shared<unsigned int>()),
    m_nMeasure(std::make_shared<unsigned int>()),
    m_xp(std::make_shared<biorbd::utils::Vector>()),
    m_A(std::make_shared<biorbd::utils::Matrix>()),
    m_Q(std::make_shared<biorbd::utils::Matrix>()),
    m_R(std::make_shared<biorbd::utils::Matrix>()),
    m_Pp(std::make_shared<biorbd::utils::Matrix>())
{

}

biorbd::rigidbody::KalmanRecons::KalmanRecons(biorbd::Model &model,
        unsigned int nbMeasure,
        KalmanParam params) :
    m_params(std::make_shared<KalmanParam>(params)),
    m_Te(std::make_shared<double>(1.0/(m_params->acquisitionFrequency()))),
    m_nbDof(std::make_shared<unsigned int>(model.dof_count)),
    m_nMeasure(std::make_shared<unsigned int>(nbMeasure)),
    m_xp(std::make_shared<biorbd::utils::Vector>()),
    m_A(std::make_shared<biorbd::utils::Matrix>()),
    m_Q(std::make_shared<biorbd::utils::Matrix>()),
    m_R(std::make_shared<biorbd::utils::Matrix>()),
    m_Pp(std::make_shared<biorbd::utils::Matrix>())
{

}

biorbd::rigidbody::KalmanRecons::~KalmanRecons()
{

}

void biorbd::rigidbody::KalmanRecons::DeepCopy(const
        biorbd::rigidbody::KalmanRecons &other)
{
    *m_params = *other.m_params;
    *m_Te = *other.m_Te;
    *m_nbDof = *other.m_nbDof;
    *m_nMeasure = *other.m_nMeasure;
    *m_xp = *other.m_xp;
    *m_A = *other.m_A;
    *m_Q = *other.m_Q;
    *m_R = *other.m_R;
    *m_Pp = *other.m_Pp;
}

void biorbd::rigidbody::KalmanRecons::iteration(
    biorbd::utils::Vector measure,
    const biorbd::utils::Vector &projectedMeasure,
    const biorbd::utils::Matrix &Hessian,
    const std::vector<unsigned int> &occlusion)
{
    // Prediction
    const biorbd::utils::Vector& xkm(*m_A * *m_xp);
    const biorbd::utils::Matrix& Pkm(*m_A * *m_Pp * m_A->transpose() + *m_Q);

    // Correction
    biorbd::utils::Matrix InvTp( (Hessian * Pkm * Hessian.transpose() +
                                  *m_R).inverse() );
    manageOcclusionDuringIteration(InvTp, measure, occlusion);
    const biorbd::utils::Matrix& K(Pkm*Hessian.transpose() * InvTp); // Gain

    *m_xp = xkm+K*(measure-projectedMeasure); // New estimated state
    const RigidBodyDynamics::Math::MatrixNd& temp(biorbd::utils::Matrix::Identity(
                3* *m_nbDof, 3* *m_nbDof) - K*Hessian);
    *m_Pp =  temp * Pkm * temp.transpose() + K* *m_R*K.transpose();

}

void biorbd::rigidbody::KalmanRecons::manageOcclusionDuringIteration(
    biorbd::utils::Matrix &InvTp,
    biorbd::utils::Vector &measure,
    const std::vector<unsigned int> &occlusion)
{
    for (unsigned int i = 0; i < occlusion.size(); ++i)
        for (unsigned int j=occlusion[i]; j< occlusion[i]+1; ++j) {
            InvTp(j,j) =
                0; // Artifact due to the fact that m_R has a value at (j:j+2,j:j+2)
            measure(j) = 0;
        }
}

void biorbd::rigidbody::KalmanRecons::getState(
    biorbd::rigidbody::GeneralizedCoordinates *Q,
    biorbd::rigidbody::GeneralizedVelocity *Qdot,
    biorbd::rigidbody::GeneralizedAcceleration *Qddot)
{

    if (Q != nullptr) {
        *Q = m_xp->block(0, 0, *m_nbDof, 1);
    }

    if (Qdot != nullptr) {
        *Qdot = m_xp->block(*m_nbDof, 0, *m_nbDof, 1);
    }

    if (Qddot != nullptr) {
        *Qddot = m_xp->block(2* *m_nbDof, 0, *m_nbDof, 1);
    }
}


biorbd::utils::Matrix biorbd::rigidbody::KalmanRecons::evolutionMatrix(
    const unsigned int nQ,
    unsigned int n,
    double Te)
{
    // m  : number of degrees of freedom
    // n  : order of Taylor development
    // Te : 1 / (acquisition frequency)

    n += 1;
    biorbd::utils::Matrix A(biorbd::utils::Matrix::Identity(nQ*n, nQ*n));
    double c = 1;
    for (unsigned int i=2; i<n+1; ++i) {

        unsigned int j=(i-1) * nQ;
        c = c/(i-1);


        for (unsigned int cmp=0; cmp<nQ*n-j; ++cmp) {
            A(0+cmp,j+cmp) += c* static_cast<double>(std::pow(Te,
                              (static_cast<double>(i)-1.0)));
        }
    }

    return A;
}

biorbd::utils::Matrix biorbd::rigidbody::KalmanRecons::processNoiseMatrix(
    const unsigned int nbQ,
    double Te)
{

    // Find the coefficients values
    double c1 = 1.0/20.0 * pow(Te,5);
    double c2 = 1.0/8.0  * pow(Te,4);
    double c3 = 1.0/6.0  * pow(Te,3);
    double c4 = 1.0/3.0  * pow(Te,3);
    double c5 = 1.0/2.0  * pow(Te,2);
    double c6 = Te;

    // Ouput matrix
    biorbd::utils::Matrix Q(biorbd::utils::Matrix::Zero(3*nbQ,3*nbQ));
    for (unsigned int j=0; j<nbQ; ++j) {
        Q(     j,      j) = c1;
        Q(     j,   nbQ+j) = c2;
        Q(     j, 2*nbQ+j) = c3;
        Q(  nbQ+j,      j) = c2;
        Q(  nbQ+j,   nbQ+j) = c4;
        Q(  nbQ+j, 2*nbQ+j) = c5;
        Q(2*nbQ+j,      j) = c3;
        Q(2*nbQ+j,   nbQ+j) = c5;
        Q(2*nbQ+j, 2*nbQ+j) = c6;
    }

    return Q;
}

// Methods during the initialization
void biorbd::rigidbody::KalmanRecons::initialize()
{

    // Declaration of the evolution matrix
    *m_A = evolutionMatrix(*m_nbDof, 2, *m_Te);

    // Process Noise Matrix
    *m_Q = processNoiseMatrix(*m_nbDof, *m_Te);

    // Matrix of the noise on the measurement
    *m_R = measurementNoiseMatrix(*m_nMeasure, m_params->noiseFactor());

    // Initialize the state
    *m_xp = initState(*m_nbDof);

    // Matrix Pp
    *m_Pp = initCovariance(*m_nbDof, m_params->errorFactor());
}


biorbd::utils::Matrix biorbd::rigidbody::KalmanRecons::measurementNoiseMatrix(
    const unsigned int nbT,
    double val)
{
    biorbd::utils::Matrix R(biorbd::utils::Matrix::Zero(nbT, nbT));
    for (unsigned int i=0; i<nbT; ++i) {
        R(i,i) = val;
    }
    return R;
}

biorbd::rigidbody::GeneralizedCoordinates
biorbd::rigidbody::KalmanRecons::initState(
    const unsigned int nbQ)
{
    return biorbd::rigidbody::GeneralizedCoordinates::Zero(3*nbQ); // Q, Qdot, Qddot
}

void biorbd::rigidbody::KalmanRecons::setInitState(
    const biorbd::rigidbody::GeneralizedCoordinates *Q,
    const biorbd::rigidbody::GeneralizedVelocity *Qdot,
    const biorbd::rigidbody::GeneralizedAcceleration *Qddot)
{
    if (Q != nullptr) {
        m_xp->block(0, 0, *m_nbDof, 1) = *Q;
    }

    if (Qdot != nullptr) {
        m_xp->block(*m_nbDof, 0, *m_nbDof, 1) = *Qdot;
    }

    if (Qddot != nullptr) {
        m_xp->block(2* *m_nbDof, 0, *m_nbDof, 1) = *Qddot;
    }
}


biorbd::utils::Matrix biorbd::rigidbody::KalmanRecons::initCovariance(
    const unsigned int nbQ,
    double val)
{
    biorbd::utils::Matrix Pp(biorbd::utils::Matrix::Zero(3*nbQ, 3*nbQ));
    for (unsigned int i=0; i<3*nbQ; ++i) {
        Pp(i,i) = val;
    }
    return Pp;
}





biorbd::rigidbody::KalmanParam::KalmanParam(
    double frequency,
    double noiseFactor,
    double errorFactor):
    m_acquisitionFrequency(frequency),
    m_noiseFactor(noiseFactor),
    m_errorFactor(errorFactor) {}

double biorbd::rigidbody::KalmanParam::acquisitionFrequency() const
{
    return m_acquisitionFrequency;
}

double biorbd::rigidbody::KalmanParam::noiseFactor() const
{
    return m_noiseFactor;
}

double biorbd::rigidbody::KalmanParam::errorFactor() const
{
    return m_errorFactor;
}
