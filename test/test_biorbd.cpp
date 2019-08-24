#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/Dynamics.h>

#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedTorque.h"

#ifdef MODULE_ACTUATORS
static std::string modelPathForGeneralTesting("models/pyomecaman_withActuators.bioMod");
#else // MODULE_ACTUATORS
static std::string modelPathForGeneralTesting("models/pyomecaman.bioMod");
#endif // MODULE_ACTUATORS
TEST(FileIO, OpenModel){
    EXPECT_NO_THROW(biorbd::Model model(modelPathForGeneralTesting));
}

TEST(GenericTests, mass){
    biorbd::Model model(modelPathForGeneralTesting);
    EXPECT_DOUBLE_EQ(model.mass(), 52.412120000000002);
}
