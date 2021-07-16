#ifndef BIORBD_MUSCLES_STATIC_OPTIMIZATION_H
#define BIORBD_MUSCLES_STATIC_OPTIMIZATION_H

#include <vector>
#include <memory>
#include <IpTNLP.hpp>
#include "biorbdConfig.h"

namespace biorbd
{
class Model;

namespace utils
{
class Vector;
}

namespace rigidbody
{
class GeneralizedCoordinates;
class GeneralizedVelocity;
class GeneralizedTorque;
}

namespace muscles
{
class StateDynamics;

///
/// \brief Base class for a Static Optimization algorithm
///
class BIORBD_API StaticOptimization
{
public:
    ///
    /// \brief Construct static optimization
    /// \param model The musculoskeletal Model
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param torqueTarget The generalized torque target to match during the optimization
    /// \param initialActivationGuess The initial activation guess
    /// \param pNormFactor The p-norm to perform
    /// \param useResidualTorque If use residual torque, if set to false, the optimization will fail if the model is not strong enough
    /// \param verbose Level of IPOPT verbose you want
    ///
    StaticOptimization(
        biorbd::Model& model,
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedVelocity& Qdot,
        const biorbd::rigidbody::GeneralizedTorque& torqueTarget,
        double initialActivationGuess = 0.01,
        unsigned int pNormFactor = 2,
        bool useResidualTorque = true,
        int verbose = 0);

    ///
    /// \brief Construct static optimization
    /// \param model The musculoskeletal Model
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param torqueTarget The generalized torque target to match during the optimization
    /// \param initialActivationGuess The initial activation guess
    /// \param pNormFactor The p-norm to perform
    /// \param useResidualTorque If use residual torque, if set to false, the optimization will fail if the model is not strong enough
    /// \param verbose Level of IPOPT verbose you want
    ///
    StaticOptimization(
        biorbd::Model& model,
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedVelocity& Qdot,
        const biorbd::rigidbody::GeneralizedTorque& torqueTarget,
        const biorbd::utils::Vector& initialActivationGuess,
        unsigned int pNormFactor = 2,
        bool useResidualTorque = true,
        int verbose = 0);

    ///
    /// \brief Construct static optimization
    /// \param model The musculoskeletal Model
    /// \param Q The generalized coordinates
    /// \param Qdot The generalized velocities
    /// \param torqueTarget The generalized torque target to match during the optimization
    /// \param initialActivationGuess The initial activation guess
    /// \param pNormFactor The p-norm to perform
    /// \param useResidualTorque If use residual torque, if set to false, the optimization will fail if the model is not strong enough
    /// \param verbose Level of IPOPT verbose you want
    ///
    StaticOptimization(
        biorbd::Model& model,
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedVelocity& Qdot,
        const biorbd::rigidbody::GeneralizedTorque& torqueTarget,
        const std::vector<biorbd::muscles::StateDynamics>& initialActivationGuess,
        unsigned int pNormFactor = 2,
        bool useResidualTorque = true,
        int verbose = 0);

    ///
    /// \brief Construct static optimization for multiple frames
    /// \param model The musculoskeletal Model
    /// \param allQ The generalized coordinates
    /// \param allQdot The generalized velocities
    /// \param allTorqueTarget The generalized torque target to match during the optimization
    /// \param initialActivationGuess The initial activation guess
    /// \param pNormFactor The p-norm to perform
    /// \param useResidualTorque If use residual torque, if set to false, the optimization will fail if the model is not strong enough
    /// \param verbose Level of IPOPT verbose you want
    ///
    StaticOptimization(
        biorbd::Model& model,
        const std::vector<biorbd::rigidbody::GeneralizedCoordinates>& allQ,
        const std::vector<biorbd::rigidbody::GeneralizedVelocity>& allQdot,
        const std::vector<biorbd::rigidbody::GeneralizedTorque>& allTorqueTarget,
        double initialActivationGuess = 0.01,
        unsigned int pNormFactor = 2,
        bool useResidualTorque = true,
        int verbose = 0);

    ///
    /// \brief Construct static optimization for multiple frames
    /// \param model The musculoskeletal Model
    /// \param allQ The generalized coordinates
    /// \param allQdot The generalized velocities
    /// \param allTorqueTarget The generalized torque target to match during the optimization
    /// \param initialActivationGuess The initial activation guess
    /// \param pNormFactor The p-norm to perform
    /// \param useResidualTorque If use residual torque, if set to false, the optimization will fail if the model is not strong enough
    /// \param verbose Level of IPOPT verbose you want
    ///
    StaticOptimization(
        biorbd::Model& model,
        const std::vector<biorbd::rigidbody::GeneralizedCoordinates>& allQ,
        const std::vector<biorbd::rigidbody::GeneralizedVelocity>& allQdot,
        const std::vector<biorbd::rigidbody::GeneralizedTorque>& allTorqueTarget,
        const biorbd::utils::Vector& initialActivationGuess,
        unsigned int pNormFactor = 2,
        bool useResidualTorque = true,
        int verbose = 0);

    ///
    /// \brief Construct static optimization for multiple frames
    /// \param model The musculoskeletal Model
    /// \param allQ The generalized coordinates
    /// \param allQdot The generalized velocities
    /// \param allTorqueTarget The generalized torque target to match during the optimization
    /// \param initialActivationGuess The initial activation guess
    /// \param pNormFactor The p-norm to perform
    /// \param useResidualTorque If use residual torque, if set to false, the optimization will fail if the model is not strong enough
    /// \param verbose Level of IPOPT verbose you want
    ///
    StaticOptimization(
        biorbd::Model& model,
        const std::vector<biorbd::rigidbody::GeneralizedCoordinates>& allQ,
        const std::vector<biorbd::rigidbody::GeneralizedVelocity>& allQdot,
        const std::vector<biorbd::rigidbody::GeneralizedTorque>& allTorqueTarget,
        const std::vector<biorbd::muscles::StateDynamics>& initialActivationGuess,
        unsigned int pNormFactor = 2,
        bool useResidualTorque = true,
        int verbose = 0);

    ///
    /// \brief Run the static optimization
    /// \param useLinearizedState If use the algorithm should be run with the linearized approach (faster but less precise)
    ///
    void run(bool useLinearizedState = true);

    ///
    /// \brief Return the final solution
    /// \return The final solution
    ///
    std::vector<biorbd::utils::Vector> finalSolution();

    ///
    /// \brief Return the final solution at a specific index
    /// \return The final solution at a specific index
    ///
    biorbd::utils::Vector finalSolution(unsigned int index);

protected:
    biorbd::Model& m_model; ///< A reference to the model
    bool m_useResidualTorque; ///< To use residual torque
    std::vector<biorbd::rigidbody::GeneralizedCoordinates>
    m_allQ; ///< All the generalized coordinates
    std::vector<biorbd::rigidbody::GeneralizedVelocity>
    m_allQdot; ///< All the generalized velocities
    std::vector<biorbd::rigidbody::GeneralizedTorque>
    m_allTorqueTarget; ///< All the torque targets
    std::shared_ptr<biorbd::utils::Vector>
    m_initialActivationGuess; ///< Initial activation guess
    unsigned int m_pNormFactor; ///< The p-norm factor
    int m_verbose; ///<Verbose level
    std::vector<Ipopt::SmartPtr<Ipopt::TNLP>>
                                           m_staticOptimProblem; ///<The static optimization problem
    bool m_alreadyRun; ///< If already ran the static optimization

};

}
}

#endif // BIORBD_MUSCLES_STATIC_OPTIMIZATION_H
