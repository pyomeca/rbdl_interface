#include "biorbd.h"
#include "Muscles/StaticOptimization.h"

///
/// \brief main Compute the muscle activations that produce a given generalized forces (Tau)
/// \return Nothing
///
/// This examples shows how to
///     1. Load a model with muscles
///     2. Position the model at a chosen position (Q) and velocity (Qdot)
///     3. Define a target generalized forces (Tau)
///     4. Compute the muscle activations that reproduce this Tau (Static optimization)
///     5. Print them to the console
///
/// Please note that this example will work only with the Eigen backend
///
int main()
{
    // Load a predefined model
    biorbd::Model model("arm26.bioMod");

    // Choose a position/velocity/torque to compute muscle activations from
    // If multiple frames, one can use std::vector of corresponding Generalized
    // to run them all.
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    Q.setZero();
    Qdot.setZero();
    Tau.setZero();

    // Proceed with the static optimization
    auto optim = biorbd::muscles::StaticOptimization(model, Q, Qdot, Tau);
    optim.run();
    auto muscleActivationsPerFrame = optim.finalSolution();

    // Print them to the console
    for (auto activations : muscleActivationsPerFrame) {
        std::cout << activations.transpose() << std::endl;
    }

    return 0;
}
