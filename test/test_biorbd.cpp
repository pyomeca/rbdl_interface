#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/Dynamics.h>

#include "BiorbdModel.h"
#include "RigidBody/Joints.h"
#include "ModelWriter.h"
#include "biorbdConfig.h"
#include "Utils/String.h"
#include "Utils/RotoTrans.h"
#include "Utils/RotoTransNode.h"
#include "RigidBody/Segment.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/IMU.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedTorque.h"

static double requiredPrecision(1e-10);

static std::string modelPathWithMeshFile("models/simpleWithMeshFile.bioMod");
#ifdef MODULE_ACTUATORS
    static std::string
    modelPathForGeneralTesting("models/pyomecaman_withActuators.bioMod");
#else // MODULE_ACTUATORS
    static std::string modelPathForGeneralTesting("models/pyomecaman.bioMod");
#endif // MODULE_ACTUATORS
static std::string modelFreeFall("models/pyomecaman_freeFall.bioMod");

static std::string modelPathWithObj("models/violin.bioMod");
#ifdef MODULE_VTP_FILES_READER
    static std::string modelPathWithVtp("models/thoraxWithVtp.bioMod");
#endif

TEST(FileIO, OpenModel)
{
    EXPECT_NO_THROW(biorbd::Model model(modelPathForGeneralTesting));
}

#ifndef BIORBD_USE_CASADI_MATH
TEST(FileIO, WriteModel)
{
    biorbd::Model model("models/two_segments.bioMod");
    biorbd::utils::String savePath("temporary.bioMod");
    biorbd::Writer::writeModel(model, savePath);
    biorbd::Model modelCopy(savePath);

    // Test if the model is properly written
    biorbd::rigidbody::GeneralizedCoordinates Q(modelCopy);
    Q.setOnes();

    for (unsigned int k=0; k<model.nbSegment(); ++k) {
        for (unsigned int i=0; i<4; ++i) {
            for (unsigned int j=0; j<4; ++j) {
                EXPECT_NEAR(model.globalJCS(Q, k)(i, j), modelCopy.globalJCS(Q, k)(i, j), 1e-5);
            }
        }
    }

    EXPECT_EQ(modelCopy.nbMarkers(), 1);
    for (unsigned int k=0; k<modelCopy.nbMarkers(); ++k) {
        for (unsigned int i=0; i<3; ++i) {
            EXPECT_NEAR(model.marker(Q, k)[i], modelCopy.marker(Q, k)[i], 1e-5);
        }
    }

    EXPECT_EQ(modelCopy.nbRTs(), 1);
    for (unsigned int k=0; k<modelCopy.nbRTs(); ++k) {
        for (unsigned int i=0; i<4; ++i) {
            for (unsigned int j=0; j<4; ++j) {
                EXPECT_NEAR(model.RT(Q, k)(i, j), modelCopy.RT(Q, k)(i, j), 1e-5);
            }
        }
    }

    EXPECT_EQ(modelCopy.nbIMUs(), 1);
    for (unsigned int k=0; k<modelCopy.nbIMUs(); ++k) {
        for (unsigned int i=0; i<4; ++i) {
            for (unsigned int j=0; j<4; ++j) {
                EXPECT_NEAR(model.IMU(Q)[k](i, j), modelCopy.IMU(Q)[k](i, j), 1e-5);
            }
        }
    }
    remove(savePath.c_str());
}
#endif

TEST(GenericTests, mass)
{
    biorbd::Model model(modelPathForGeneralTesting);
    SCALAR_TO_DOUBLE(mass, model.mass());
    EXPECT_NEAR(mass, 52.41212, requiredPrecision);
}

TEST(MeshFile, FileIO)
{
    EXPECT_NO_THROW(biorbd::Model model(modelPathWithMeshFile));
    biorbd::Model model(modelPathWithMeshFile);
}

TEST(MeshFile, FileIoObj)
{
    EXPECT_NO_THROW(biorbd::Model model(modelPathWithObj));
    biorbd::Model model(modelPathWithObj);
}

#ifdef MODULE_VTP_FILES_READER
TEST(MeshFile, FileIoVtp)
{
    EXPECT_NO_THROW(biorbd::Model model(modelPathWithVtp));
    biorbd::Model model(modelPathWithVtp);
}
#endif
