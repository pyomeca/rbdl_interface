#include "biorbd.h"

///
/// \brief main Get and print the generalized acceleration of a model
/// \return Nothing
///
/// This examples shows how to
///     1. Load a model
///     2. Position the model at a chosen position (Q), velocity (Qdot)
///     3. Compute the generalized acceleration (Qddot) assuming a set
///        muscle activations (joint torque from muscle)
///     4. Print them to the console
///
/// Please note that this example will work only with the Eigen backend
///
int main()
{
    // Load a predefined model
    biorbd::Model model("arm26.bioMod");

    // Choose a position/velocity to compute dynamics from
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    Q.setZero();
    Qdot.setZero();

    // Set all muscles to half of their maximal activation
    std::vector<std::shared_ptr<biorbd::muscles::State>> states = model.stateSet();
    for (unsigned int i=0; i<model.nbMuscles(); ++i) {
        states[i]->setExcitation(0);
        states[i]->setActivation(0.5);
    }

    // Proceed with the computation of joint torque from the muscles
    auto Tau = model.muscularJointTorque(states, Q, Qdot);

    // Compute the generalized accelerations using the Tau from muscles.
    // Please note that in forward dynamics setting, it is usually advised to
    // additionnal residual torques. You would add them here to Tau.
    auto Qddot = model.ForwardDynamics(Q, Qdot, Tau);

    // Print them to the console
    std::cout << Qddot.transpose() << std::endl;

    // As an extra, let's print the individual muscle forces
    std::cout << model.muscleForces(states, Q, Qdot).transpose() << std::endl;

    return 0;
}
