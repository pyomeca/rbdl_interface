#include "biorbd.h"

///
/// \brief main Get and print the position of all markers of a model
/// \return Nothing
///
/// This examples shows how to
///     1. Load a model
///     2. Position the model at a chosen position (Q)
///     3. Compute the position of the skin markers at that position (Forward kinematics)
///     4. Print them to the console
///
/// Please note that this example will work only with the Eigen backend
///
int main()
{
    // Load a predefined model
    biorbd::Model model("pyomecaman.bioMod");

    // Choose a position to get the markers from
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    Q.setZero();

    // Proceed with the forward kinematics
    auto markers = model.markers(Q);

    // Print them to the console
    for (auto marker : markers) {
        std::cout << marker.transpose() << std::endl;
    }

    return 0;
}
