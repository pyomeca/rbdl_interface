#include "biorbd.h"

///
/// \brief main Get and print the generalized acceleration of a model with wrapping objects
/// \return Nothing
///
/// This examples shows how to
///     1. Load a model with a wrapping object
///     2. Position the model at a chosen position (Q), velocity (Qdot)
///     3. Compute the generalized acceleration (Qddot) assuming a set of muscle activations
///     4. Print them to the console
///
/// Please note that this example will work only with the Eigen backend
///
int main()
{
    // Load a predefined model
    biorbd::Model model("WrappingObjectExample.bioMod");

    // Choose a position/velocity to compute dynamics from
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    Q = Q.setOnes()/10;
    Qdot.setZero();

    // Set all muscles to half of their maximal activation
    std::vector<std::shared_ptr<biorbd::muscles::State>> states;
    for (unsigned int i=0; i<model.nbMuscles(); ++i) {
        states.push_back(std::make_shared<biorbd::muscles::State>(0, 0.5));
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
