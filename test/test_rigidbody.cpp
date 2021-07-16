#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/rbdl_math.h>
#include <rbdl/Dynamics.h>

#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "Utils/String.h"
#include "Utils/Range.h"
#include "Utils/SpatialVector.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/Mesh.h"
#include "RigidBody/SegmentCharacteristics.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/Segment.h"
#include "RigidBody/IMU.h"
#ifdef MODULE_KALMAN
    #include "RigidBody/KalmanReconsMarkers.h"
    #include "RigidBody/KalmanReconsIMU.h"
#endif


static double requiredPrecision(1e-10);
#ifdef MODULE_ACTUATORS
    static std::string
    modelPathForGeneralTesting("models/pyomecaman_withActuators.bioMod");
#else // MODULE_ACTUATORS
    static std::string modelPathForGeneralTesting("models/pyomecaman.bioMod");
#endif // MODULE_ACTUATORS
static std::string modelPathMeshEqualsMarker("models/meshsEqualMarkers.bioMod");
static std::string
modelPathForLoopConstraintTesting("models/loopConstrainedModel.bioMod");
static std::string modelNoRoot("models/pyomecaman_freeFall.bioMod");
static std::string modelSimple("models/cube.bioMod");

TEST(Gravity, change)
{
    biorbd::Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_GENERALIZED_TORQUE(Tau, model);
    // Set to random values
    std::vector<double> val(model.nbQ());
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    FILL_VECTOR(Tau, val);
    {
        CALL_BIORBD_FUNCTION_3ARGS(QDDot, model, ForwardDynamics, Q, QDot, Tau);

        std::vector<double> QDDot_expected(13);
        QDDot_expected[1] = -9.81;
        for (unsigned int i = 0; i<model.nbQddot(); ++i) {
            EXPECT_NEAR(static_cast<double>(QDDot(i, 0)), QDDot_expected[i],
                        requiredPrecision);
        }
    }

    model.setGravity(biorbd::utils::Vector3d(0, -2.2, 0));
    {
        CALL_BIORBD_FUNCTION_3ARGS(QDDot, model, ForwardDynamics, Q, QDot, Tau);
        std::vector<double> QDDot_expected(13);
        QDDot_expected[0] = -2.2;
        for (unsigned int i = 0; i<model.nbQddot(); ++i) {
            EXPECT_NEAR(static_cast<double>(QDDot(i, 0)), QDDot_expected[i],
                        requiredPrecision);
        }
    }
}

TEST(Contacts, unitTest)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Contacts contacts(model);

        EXPECT_NEAR(contacts.nbContacts(), 6., requiredPrecision);
        EXPECT_STREQ(contacts.contactName(1).c_str(), "PiedG_1_Z");
        EXPECT_STREQ(contacts.contactNames()[1].c_str(), "PiedG_1_Z");
        EXPECT_EQ(contacts.hasContacts(), true);
        SCALAR_TO_DOUBLE(fy, contacts.getForce()[1]);
        EXPECT_NEAR(fy, 0., requiredPrecision);
    }
    {
        biorbd::rigidbody::Contacts contacts;
        EXPECT_EQ(contacts.hasContacts(), false);
    }
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Contacts contacts(model);

        EXPECT_THROW(contacts.contactName(7), std::runtime_error);
    }
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Contacts contacts(model);

        EXPECT_NEAR(contacts.nbContacts(), 6., requiredPrecision);

        contacts.AddConstraint(
            7,
            biorbd::utils::Vector3d(0, 0, 0),
            biorbd::utils::Vector3d(1, 1, 1),
            "constraintName",
            2.0);

        EXPECT_NEAR(contacts.nbContacts(), 7., requiredPrecision);
    }
}

TEST(Contacts, DeepCopy)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Contacts contacts(model);

        biorbd::rigidbody::Contacts shallowCopy(contacts);
        biorbd::rigidbody::Contacts deepCopyNow(contacts.DeepCopy());
        biorbd::rigidbody::Contacts deepCopyLater;
        deepCopyLater.DeepCopy(contacts);

        EXPECT_NEAR(contacts.nbContacts(), 6., requiredPrecision);
        EXPECT_NEAR(shallowCopy.nbContacts(), 6., requiredPrecision);
        EXPECT_NEAR(deepCopyNow.nbContacts(), 6., requiredPrecision);
        EXPECT_NEAR(deepCopyLater.nbContacts(), 6., requiredPrecision);

        contacts.AddConstraint(
            7,
            biorbd::utils::Vector3d(0, 0, 0),
            biorbd::utils::Vector3d(1, 1, 1),
            "constraintName",
            2.0);

        EXPECT_NEAR(contacts.nbContacts(), 7., requiredPrecision);
        EXPECT_NEAR(shallowCopy.nbContacts(), 7., requiredPrecision);
        EXPECT_NEAR(deepCopyNow.nbContacts(), 6., requiredPrecision);
        EXPECT_NEAR(deepCopyLater.nbContacts(), 6., requiredPrecision);
    }
}

static std::vector<double> Qtest = { 0.1, 0.1, 0.1, 0.3, 0.3, 0.3,
                                     0.3, 0.3, 0.3, 0.3, 0.3, 0.4, 0.3
                                   };

TEST(GeneralizedCoordinates, unitTest)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        for (unsigned int i = 0; i < model.nbQ(); ++i) {
            Q[i] = Qtest[i];
        }

        biorbd::rigidbody::GeneralizedCoordinates newQ(Q);

        std::vector<double> Q_expected = { 0.1, 0.1, 0.1, 0.3, 0.3, 0.3,
                                           0.3, 0.3, 0.3, 0.3, 0.3, 0.4, 0.3
                                         };

        for (unsigned int i = 0; i < model.nbQ(); ++i) {
            SCALAR_TO_DOUBLE(q, newQ[i]);
            EXPECT_NEAR(q, Q_expected[i], requiredPrecision);
        }
    }
}

TEST(GeneralizedVelocity, unitTest)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::GeneralizedVelocity Qdot(model);
        for (unsigned int i = 0; i < model.nbQdot(); ++i) {
            Qdot[i] = Qtest[i] * 10;
        }

        std::vector<double> Qdot_expected = { 1., 1., 1., 3., 3., 3.,
                                              3., 3., 3., 3., 3., 4., 3.
                                            };

        biorbd::rigidbody::GeneralizedVelocity newQdot(Qdot);

        for (unsigned int i = 0; i < model.nbQdot(); ++i) {
            SCALAR_TO_DOUBLE(qdot, newQdot[i]);
            EXPECT_NEAR(qdot, Qdot_expected[i], requiredPrecision);
        }
    }
    {
        biorbd::rigidbody::GeneralizedVelocity Qdot;
        SCALAR_TO_DOUBLE(qdotNorm, Qdot.norm());
        EXPECT_NEAR(qdotNorm, 0., requiredPrecision);

        biorbd::rigidbody::GeneralizedVelocity newQdot(Qdot);
        SCALAR_TO_DOUBLE(newQdotNorm, newQdot.norm());
        EXPECT_NEAR(newQdotNorm, 0., requiredPrecision);
    }
}

TEST(GeneralizedAcceleration, unitTest)
{
    {
        biorbd::rigidbody::GeneralizedAcceleration Qddot;
        SCALAR_TO_DOUBLE(QddotNorm, Qddot.norm());
        EXPECT_NEAR(QddotNorm, 0., requiredPrecision);
    }
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::GeneralizedAcceleration Qddot(model);
        for (unsigned int i = 0; i < model.nbQ(); ++i) {
            Qddot[i] = Qtest[i] * 100;
        }

        std::vector<double> Qddot_expected = { 10., 10., 10., 30., 30., 30.,
                                               30., 30., 30., 30., 30., 40., 30.
                                             };

        biorbd::rigidbody::GeneralizedAcceleration newQddot(Qddot);

        for (unsigned int i = 0; i < model.nbQ(); ++i) {
            SCALAR_TO_DOUBLE(qddot, newQddot[i]);
            EXPECT_NEAR(qddot, Qddot_expected[i], requiredPrecision);
        }
    }
}

TEST(GeneralizedTorque, unitTest)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::GeneralizedTorque Tau(model);

#ifdef BIORBD_USE_EIGEN3_MATH
        Tau << 0.1, 0.1, 0.1, 0.3, 0.3, 0.3,
            0.3, 0.3, 0.3, 0.3, 0.3, 0.4, 0.3;
#else
        Tau(0, 0) = 0.1;
        Tau(1, 0) = 0.1;
        Tau(2, 0) = 0.1;
        Tau(3, 0) = 0.3;
        Tau(4, 0) = 0.3;
        Tau(5, 0) = 0.3;
        Tau(6, 0) = 0.3;
        Tau(7, 0) = 0.3;
        Tau(8, 0) = 0.3;
        Tau(9, 0) = 0.3;
        Tau(10, 0) = 0.3;
        Tau(11, 0) = 0.4;
        Tau(12, 0) = 0.3;
#endif

        std::vector<double> Tau_expected = { 0.1, 0.1, 0.1, 0.3, 0.3, 0.3,
                                             0.3, 0.3, 0.3, 0.3, 0.3, 0.4, 0.3
                                           };

        for (unsigned int i = 0; i < 12; ++i) {
            SCALAR_TO_DOUBLE(tau, Tau[i]);
            EXPECT_NEAR(tau, Tau_expected[i], requiredPrecision);
        }

        biorbd::rigidbody::GeneralizedTorque newTau(Tau);
        for (unsigned int i = 0; i < 12; ++i) {
            SCALAR_TO_DOUBLE(tau, newTau[i]);
            EXPECT_NEAR(tau, Tau_expected[i], requiredPrecision);
        }

        biorbd::rigidbody::GeneralizedTorque tauWithNoArgument;
        SCALAR_TO_DOUBLE(tauWithNoArgumentNorm, tauWithNoArgument.norm());
        EXPECT_NEAR(tauWithNoArgumentNorm, 0., requiredPrecision);
    }
}

TEST(IMU, unitTest)
{
    biorbd::rigidbody::IMU imu(true, false);
    EXPECT_EQ(imu.isTechnical(), true);
    EXPECT_EQ(imu.isAnatomical(), false);
}

TEST(IMU, DeepCopy)
{
    biorbd::rigidbody::IMU imu(false, true);

    biorbd::rigidbody::IMU shallowCopy(imu);
    biorbd::rigidbody::IMU deepCopyNow(imu.DeepCopy());
    biorbd::rigidbody::IMU deepCopyLater;
    deepCopyLater.DeepCopy(imu);

    EXPECT_EQ(shallowCopy.isTechnical(), false);
    EXPECT_EQ(deepCopyNow.isTechnical(), false);
    EXPECT_EQ(deepCopyLater.isTechnical(), false);
}

static std::string
modelPathForPyomecaman_withIMUs("models/IMUandCustomRT/pyomecaman_withIMUs.bioMod");
TEST(IMUs, unitTest)
{
    {
        biorbd::rigidbody::IMUs imus;
        imus.addIMU(true, true);

        EXPECT_EQ(imus.nbIMUs(), 1);
        EXPECT_EQ(imus.IMU(0).isTechnical(), true);
    }
    {
        biorbd::rigidbody::IMUs imus;
        biorbd::rigidbody::IMU imu(true, false);
        imu.setName("imuName");
        imus.addIMU(imu);

        EXPECT_STREQ(imus.technicalIMU()[0].biorbd::utils::Node::name().c_str(),
                     "imuName");
    }
    {
        biorbd::Model model(modelPathForPyomecaman_withIMUs);
        biorbd::rigidbody::IMUs imus(model);

        EXPECT_EQ(imus.anatomicalIMU().size(), 2);
        EXPECT_EQ(imus.technicalIMU().size(), 4);
    }
}

TEST(IMUs, deepCopy)
{
    biorbd::Model model(modelPathForPyomecaman_withIMUs);
    biorbd::rigidbody::IMUs imus(model);

    biorbd::rigidbody::IMUs shallowCopy(imus);
    biorbd::rigidbody::IMUs deepCopyNow(imus.DeepCopy());
    biorbd::rigidbody::IMUs deepCopyLater;
    deepCopyLater.DeepCopy(imus);

    EXPECT_EQ(shallowCopy.nbIMUs(), 4);
    EXPECT_EQ(deepCopyNow.nbIMUs(), 4);
    EXPECT_EQ(deepCopyLater.nbIMUs(), 4);

    imus.addIMU(true, true);
    EXPECT_EQ(imus.nbIMUs(), 5);
    EXPECT_EQ(shallowCopy.nbIMUs(), 5);
    EXPECT_EQ(deepCopyNow.nbIMUs(), 4);
    EXPECT_EQ(deepCopyLater.nbIMUs(), 4);
}

TEST(Joints, copy)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Joints joints(model);

        biorbd::rigidbody::Joints shallowCopy(joints);
        biorbd::rigidbody::Joints deepCopyNow(joints.DeepCopy());
        biorbd::rigidbody::Joints deepCopyLater;
        deepCopyLater.DeepCopy(joints);
        {
            SCALAR_TO_DOUBLE(shallowCopyMass, shallowCopy.mass());
            SCALAR_TO_DOUBLE(deepCopyNowMass, deepCopyNow.mass());
            SCALAR_TO_DOUBLE(deepCopyLaterMass, deepCopyLater.mass());
            EXPECT_NEAR(shallowCopyMass, 52.412120000000002, requiredPrecision);
            EXPECT_NEAR(deepCopyNowMass, 52.412120000000002, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterMass, 52.412120000000002, requiredPrecision);
        }

        biorbd::rigidbody::SegmentCharacteristics characteristics(
            10, biorbd::utils::Vector3d(0.5, 0.5, 0.5),
            RigidBodyDynamics::Math::Matrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1));
        std::vector<biorbd::utils::Range> ranges(6);

        joints.AddSegment("segmentName", "parentName", "zyx", "yzx", ranges, ranges,
                          ranges,
                          characteristics, RigidBodyDynamics::Math::SpatialTransform());

        {
            SCALAR_TO_DOUBLE(jointsMass, joints.mass());
            SCALAR_TO_DOUBLE(shallowCopyMass, shallowCopy.mass());
            SCALAR_TO_DOUBLE(deepCopyNowMass, deepCopyNow.mass());
            SCALAR_TO_DOUBLE(deepCopyLaterMass, deepCopyLater.mass());
            EXPECT_NEAR(jointsMass, 62.412120000000002, requiredPrecision);
            EXPECT_NEAR(shallowCopyMass, 62.412120000000002, requiredPrecision);
            EXPECT_NEAR(deepCopyNowMass, 52.412120000000002, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterMass, 52.412120000000002, requiredPrecision);
        }
    }
}

TEST(Joints, unitTest)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Joints joints(model);
        std::vector <biorbd::utils::String> names(joints.nameDof());

        std::vector<biorbd::utils::String> expectedNames(joints.nbDof());
        expectedNames = { "Pelvis_TransY", "Pelvis_TransZ", "Pelvis_RotX",
                          "BrasD_RotZ", "BrasD_RotX", "BrasG_RotZ", "BrasG_RotX",
                          "CuisseD_RotX", "JambeD_RotX", "PiedD_RotX", "CuisseG_RotX",
                          "JambeG_RotX", "PiedG_RotX"
                        };


        for (unsigned int i = 0; i < joints.nbDof(); ++i) {
            EXPECT_STREQ(names[i].c_str(), expectedNames[i].c_str());
        }
    }
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Joints joints(model);
        biorbd::rigidbody::Segment segmentToTest(joints.segment("Tronc"));

        EXPECT_EQ(segmentToTest.id(), 2147483647); //TODO: Verify ID value
    }
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Joints joints(model);
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        biorbd::rigidbody::GeneralizedVelocity Qdot(model);
        for (size_t i=0; i<model.nbQ(); ++i) {
            Q[i] = static_cast<double>(i) * 0.2;
            Qdot[i] = static_cast<double>(i) * 1.2;
        }

        biorbd::utils::Vector3d angularMomentum(joints.angularMomentum(Q, Qdot));
        std::vector<double> expectedAngularMomentum = {15.957205552043206, -2.399856350425782, 2.0751269909741334};

        for (int i = 0; i < 3; ++i) {
            SCALAR_TO_DOUBLE(momentum, angularMomentum[i]);
            EXPECT_NEAR(momentum, expectedAngularMomentum[i], requiredPrecision);
        }
    }
}

TEST(Markers, copy)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        biorbd::rigidbody::Markers markers(model);

        biorbd::rigidbody::Markers shallowCopy(markers);
        biorbd::rigidbody::Markers deepCopyNow(markers.DeepCopy());
        biorbd::rigidbody::Markers deepCopyLater;
        deepCopyLater.DeepCopy(markers);

        EXPECT_EQ(markers.nbMarkers(), 97);
        EXPECT_EQ(shallowCopy.nbMarkers(), 97);
        EXPECT_EQ(deepCopyNow.nbMarkers(), 97);
        EXPECT_EQ(deepCopyLater.nbMarkers(), 97);


        biorbd::rigidbody::NodeSegment nodeSegment;
        markers.addMarker(nodeSegment,
                          "markerName", "parentName", true, true, "x", 98);

        EXPECT_EQ(markers.nbMarkers(), 98);
        EXPECT_EQ(shallowCopy.nbMarkers(), 98);
        EXPECT_EQ(deepCopyNow.nbMarkers(), 97);
        EXPECT_EQ(deepCopyLater.nbMarkers(), 97);
    }
}

TEST(SegmentCharacteristics, length)
{
    biorbd::rigidbody::SegmentCharacteristics segmentCharac;
    SCALAR_TO_DOUBLE(length1, segmentCharac.length());
    EXPECT_NEAR(length1, 0., requiredPrecision);
    segmentCharac.setLength(2.);
    SCALAR_TO_DOUBLE(length2, segmentCharac.length());
    EXPECT_NEAR(length2, 2., requiredPrecision);
}

TEST(Segment, nameDof)
{
    biorbd::Model model(modelPathForGeneralTesting);
    EXPECT_THROW(model.segment(128), std::runtime_error);
}

static std::string modelPathForRTsane("models/IMUandCustomRT/RT_sane.bioMod");
static std::string
modelPathForRTwrong1("models/IMUandCustomRT/RT_wrong1.bioMod");
static std::string
modelPathForRTwrong2("models/IMUandCustomRT/RT_wrong2.bioMod");
static std::string
modelPathForRTwrong3("models/IMUandCustomRT/RT_wrong3.bioMod");
static std::string
modelPathForRTwrong4("models/IMUandCustomRT/RT_wrong4.bioMod");
static std::string
modelPathForRTwrong5("models/IMUandCustomRT/RT_wrong5.bioMod");
static std::string
modelPathForRTwrong6("models/IMUandCustomRT/RT_wrong6.bioMod");
TEST(RotoTransNode, Read)
{
    {
        biorbd::Model model(modelPathForRTsane);
        biorbd::rigidbody::RotoTransNodes rt(model);

        EXPECT_EQ(rt.size(), 3);
    }

    {
        EXPECT_THROW(biorbd::Model model(modelPathForRTwrong1), std::runtime_error);
        EXPECT_THROW(biorbd::Model model(modelPathForRTwrong2), std::runtime_error);
        EXPECT_THROW(biorbd::Model model(modelPathForRTwrong3), std::runtime_error);
        EXPECT_THROW(biorbd::Model model(modelPathForRTwrong4), std::runtime_error);
        EXPECT_THROW(biorbd::Model model(modelPathForRTwrong5), std::runtime_error);
        EXPECT_THROW(biorbd::Model model(modelPathForRTwrong6), std::runtime_error);
    }
}

TEST(RotoTransNode, copy)
{
    biorbd::Model model(modelPathForGeneralTesting);
    biorbd::rigidbody::RotoTransNodes rtNode(model);
    biorbd::utils::RotoTransNode rt(
        biorbd::utils::RotoTrans(biorbd::utils::Vector3d(2, 3, 4),
                                 biorbd::utils::Vector3d(), "xyz"), "", "" );
    rtNode.addRT(rt);

    biorbd::rigidbody::RotoTransNodes shallowCopy(rtNode);
    biorbd::rigidbody::RotoTransNodes deepCopyNow(rtNode.DeepCopy());
    biorbd::rigidbody::RotoTransNodes deepCopyLater;
    deepCopyLater.DeepCopy(rtNode);

    EXPECT_EQ(shallowCopy.RTs().size(), 1);
    EXPECT_EQ(deepCopyNow.RTs().size(), 1);
    EXPECT_EQ(deepCopyLater.RTs().size(), 1);

    rtNode.addRT(rt);
    EXPECT_EQ(rtNode.RTs().size(), 2);
    EXPECT_EQ(shallowCopy.RTs().size(), 2);
    EXPECT_EQ(deepCopyNow.RTs().size(), 1);
    EXPECT_EQ(deepCopyLater.RTs().size(), 1);
}

TEST(RotoTransNode, unitTest)
{
    {
        biorbd::rigidbody::RotoTransNodes rtNode;
        biorbd::utils::RotoTransNode rt(
            biorbd::utils::RotoTrans(biorbd::utils::Vector3d(2, 3, 4),
                                     biorbd::utils::Vector3d(), "xyz"), "", "" );
        rtNode.addRT(rt);
        auto rt_vector(rtNode.RTs());
#ifndef BIORBD_USE_CASADI_MATH
        EXPECT_NEAR(rt_vector[0].norm(), 1.9999999999999998, requiredPrecision);
        EXPECT_NEAR(rtNode.RT(0).norm(), 1.9999999999999998, requiredPrecision);
#endif
    }
    {
        biorbd::rigidbody::RotoTransNodes rtNode;
        biorbd::utils::RotoTransNode rt(
            biorbd::utils::RotoTrans(biorbd::utils::Vector3d(2, 3, 4),
                                     biorbd::utils::Vector3d(), "xyz"), "", "" );
        rtNode.addRT(rt);
        std::vector<biorbd::utils::RotoTransNode> rt_vector(rtNode.RTs());
        rt_vector[0].setParent("parentName");
        rt_vector[0].setName("nameSet");
        std::vector<biorbd::utils::String> expectedNames = { "nameSet" };
        for (unsigned int i = 0; i < rt_vector.size(); ++i) {
            EXPECT_STREQ(rtNode.RTs("parentName")[i].biorbd::utils::Node::name().c_str(),
                         expectedNames[i].c_str());
            EXPECT_STREQ(rtNode.RTsNames()[i].c_str(), expectedNames[i].c_str());
        }
    }
    {
        biorbd::rigidbody::RotoTransNodes rtNode;
        rtNode.addRT();
        unsigned int numberOfRTs(rtNode.nbRTs());
        EXPECT_EQ(numberOfRTs, 1);
    }
}


TEST(NodeSegment, unitTests)
{
    {
        biorbd::rigidbody::NodeSegment nodeSegment(1.1, 2.2, 3.3);
        SCALAR_TO_DOUBLE(x, nodeSegment.x());
        SCALAR_TO_DOUBLE(y, nodeSegment.y());
        SCALAR_TO_DOUBLE(z, nodeSegment.z());
        EXPECT_NEAR(x, 1.1, requiredPrecision);
        EXPECT_NEAR(y, 2.2, requiredPrecision);
        EXPECT_NEAR(z, 3.3, requiredPrecision);
    }
    {
        biorbd::rigidbody::NodeSegment nodeSegment(biorbd::utils::Vector3d(2, 3, 4),
                "nodeSegmentName", "parentName", true, true, "z", 8);
        EXPECT_STREQ(nodeSegment.parent().c_str(), "parentName");
    }
    {
        biorbd::rigidbody::NodeSegment nodeSegment(biorbd::utils::Vector3d(2, 3, 4),
                "nodeSegmentName", "parentName", true, true, "z", 8);

        EXPECT_EQ(nodeSegment.isAxisKept(2), false);
        EXPECT_EQ(nodeSegment.isAxisRemoved(2), true);
    }
    {
        biorbd::rigidbody::NodeSegment nodeSegment(biorbd::utils::Vector3d(2, 3, 4),
                "nodeSegmentName", "parentName", true, true, "z", 8);
        std::vector<biorbd::utils::String> vector = { "x", "y" };
        nodeSegment.addAxesToRemove(vector);
        EXPECT_STREQ(nodeSegment.axesToRemove().c_str(), "xyz");
    }
    {
        biorbd::rigidbody::NodeSegment nodeSegment(biorbd::utils::Vector3d(2, 3, 4),
                "nodeSegmentName", "parentName", true, true, "z", 8);
        std::vector<unsigned int> vector = {0, 1};
        nodeSegment.addAxesToRemove(vector);
        EXPECT_STREQ(nodeSegment.axesToRemove().c_str(), "xyz");
    }
    {
        biorbd::rigidbody::NodeSegment nodeSegment(biorbd::utils::Vector3d(2, 3, 4),
                "nodeSegmentName", "parentName", true, true, "z", 8);
        EXPECT_THROW(nodeSegment.addAxesToRemove(4), std::runtime_error);
        biorbd::utils::String string("m");
        EXPECT_THROW(nodeSegment.addAxesToRemove(string), std::runtime_error);
    }
}


TEST(NodeSegment, copy)
{
    biorbd::rigidbody::NodeSegment nodeSegment(biorbd::utils::Vector3d(2, 3, 4),
            "nodeSegmentName", "parentName", true, true, "z", 8);

    biorbd::rigidbody::NodeSegment deepCopyNow(nodeSegment.DeepCopy());
    biorbd::rigidbody::NodeSegment deepCopyLater;
    deepCopyLater.DeepCopy(nodeSegment);

    EXPECT_EQ(nodeSegment.nbAxesToRemove(), 1);
    EXPECT_EQ(deepCopyNow.nbAxesToRemove(), 1);
    EXPECT_EQ(deepCopyLater.nbAxesToRemove(), 1);
}

TEST(DegreesOfFreedom, count)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        EXPECT_EQ(model.nbQ(), 13);
        EXPECT_EQ(model.nbQdot(), 13);
        EXPECT_EQ(model.nbQddot(), 13);
        EXPECT_EQ(model.nbGeneralizedTorque(), 13);
        EXPECT_EQ(model.nbRoot(), 3);
    }
    {
        biorbd::Model model(modelNoRoot);
        EXPECT_EQ(model.nbQ(), 13);
        EXPECT_EQ(model.nbQdot(), 13);
        EXPECT_EQ(model.nbQddot(), 13);
        EXPECT_EQ(model.nbGeneralizedTorque(), 13);
        EXPECT_EQ(model.nbRoot(), 3);
    }
}

TEST(DegressOfFreedom, ranges)
{
    biorbd::Model model(modelPathForGeneralTesting);
    std::vector<biorbd::utils::Range> QRanges;
    std::vector<biorbd::utils::Range> QDotRanges;
    std::vector<biorbd::utils::Range> QDDotRanges;

    auto a = model.meshPoints(biorbd::rigidbody::GeneralizedCoordinates(model));

    // Pelvis
    QRanges = model.segment(0).QRanges();
    EXPECT_EQ(QRanges[0].min(), -15);
    EXPECT_EQ(QRanges[0].max(), 15);
    EXPECT_EQ(QRanges[1].min(), -15);
    EXPECT_EQ(QRanges[1].max(), 15);
    EXPECT_EQ(QRanges[2].min(), -M_PI+1);
    EXPECT_EQ(QRanges[2].max(), M_PI+1);

    QDotRanges = model.segment(0).QDotRanges();
    EXPECT_EQ(QDotRanges[0].min(), -150);
    EXPECT_EQ(QDotRanges[0].max(), 150);
    EXPECT_EQ(QDotRanges[1].min(), -150);
    EXPECT_EQ(QDotRanges[1].max(), 150);
    EXPECT_EQ(QDotRanges[2].min(), -(M_PI+1)*10);
    EXPECT_EQ(QDotRanges[2].max(), (M_PI+1)*10);

    QDDotRanges = model.segment(0).QDDotRanges();
    EXPECT_EQ(QDDotRanges[0].min(), -1500);
    EXPECT_EQ(QDDotRanges[0].max(), 1500);
    EXPECT_EQ(QDDotRanges[1].min(), -1500);
    EXPECT_EQ(QDDotRanges[1].max(), 1500);
    EXPECT_EQ(QDDotRanges[2].min(), -(M_PI+1)*100);
    EXPECT_EQ(QDDotRanges[2].max(), (M_PI+1)*100);

    // BrasD
    QRanges = model.segment(3).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI);
    EXPECT_EQ(QRanges[0].max(), M_PI);
    EXPECT_EQ(QRanges[1].min(), 0);
    EXPECT_EQ(QRanges[1].max(), M_PI);

    QDotRanges = model.segment(3).QDotRanges();
    EXPECT_EQ(QDotRanges[0].min(), -M_PI*10);
    EXPECT_EQ(QDotRanges[0].max(), M_PI*10);
    EXPECT_EQ(QDotRanges[1].min(), -M_PI*10);
    EXPECT_EQ(QDotRanges[1].max(), M_PI*10);

    QDDotRanges = model.segment(3).QDDotRanges();
    EXPECT_EQ(QDDotRanges[0].min(), -M_PI*100);
    EXPECT_EQ(QDDotRanges[0].max(), M_PI*100);
    EXPECT_EQ(QDDotRanges[1].min(), -M_PI*100);
    EXPECT_EQ(QDDotRanges[1].max(), M_PI*100);

    // BrasG
    QRanges = model.segment(4).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI);
    EXPECT_EQ(QRanges[0].max(), M_PI);
    EXPECT_EQ(QRanges[1].min(), 0);
    EXPECT_EQ(QRanges[1].max(), M_PI);

    // CuisseD
    QRanges = model.segment(5).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI/12);
    EXPECT_EQ(QRanges[0].max(), M_PI/2+M_PI/3);

    // JambeD
    QRanges = model.segment(6).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI/2-M_PI/6);
    EXPECT_EQ(QRanges[0].max(), 0);

    // PiedD
    QRanges = model.segment(7).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI/2);
    EXPECT_EQ(QRanges[0].max(), M_PI/2);

    // CuisseG
    QRanges = model.segment(8).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI/12);
    EXPECT_EQ(QRanges[0].max(), M_PI/2+M_PI/3);

    // JambeG
    QRanges = model.segment(9).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI/2-M_PI/6);
    EXPECT_EQ(QRanges[0].max(), 0);

    // PiedG
    QRanges = model.segment(10).QRanges();
    EXPECT_EQ(QRanges[0].min(), -M_PI/2);
    EXPECT_EQ(QRanges[0].max(), M_PI/2);
}

static std::vector<double> QtestPyomecaman = {0.1, 0.1, 0.1, 0.3, 0.3, 0.3,
                                              0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3
                                             };
static std::vector<double> QtestEqualsMarker = {0.1, 0.1, 0.1, 0.3, 0.3, 0.3};

TEST(CoM, kinematics)
{
    biorbd::Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(Qdot, model);
    DECLARE_GENERALIZED_ACCELERATION(Qddot, model);

    for (unsigned int i=0; i<model.nbQ(); ++i) {
        Q(i, 0) = QtestPyomecaman[i];
        Qdot(i, 0) = QtestPyomecaman[i]*10;
        Qddot(i, 0) = QtestPyomecaman[i]*100;
    }

    CALL_BIORBD_FUNCTION_1ARG(com, model, CoM, Q);
    CALL_BIORBD_FUNCTION_2ARGS(comDot, model, CoMdot, Q, Qdot);
    CALL_BIORBD_FUNCTION_3ARGS(comDdot, model, CoMddot, Q, Qdot, Qddot);

    std::vector<double> expectedCom = {-0.0034679564024098523, 0.15680579877453169, 0.07808112642459612};
    std::vector<double> expectedComDot = {-0.05018973433722229, 1.4166208451420528, 1.4301750486035787};
    std::vector<double> expectedComDdot = {-0.7606169667295027, 11.508107073695976, 16.58853835505851};

    for (unsigned int i=0; i<3; ++i) {
        EXPECT_NEAR(static_cast<double>(com(i, 0)), expectedCom[i], requiredPrecision);
        EXPECT_NEAR(static_cast<double>(comDot(i, 0)), expectedComDot[i],
                    requiredPrecision);
        EXPECT_NEAR(static_cast<double>(comDdot(i, 0)), expectedComDdot[i],
                    requiredPrecision);
    }
}

TEST(Segment, copy)
{
    biorbd::Model model(modelPathForGeneralTesting);
    biorbd::rigidbody::SegmentCharacteristics characteristics(
        10, biorbd::utils::Vector3d(0.5, 0.5, 0.5),
        RigidBodyDynamics::Math::Matrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1));
    std::vector<biorbd::utils::Range> ranges(6);
    biorbd::rigidbody::Segment MasterSegment(
        model, "MasterSegment", "NoParent", "zyx", "yzx", ranges, ranges, ranges,
        characteristics, RigidBodyDynamics::Math::SpatialTransform());
    biorbd::rigidbody::Segment ShallowCopy(MasterSegment);
    biorbd::rigidbody::Segment ShallowCopyEqual = MasterSegment;
    biorbd::rigidbody::Segment DeepCopyNow(MasterSegment.DeepCopy());
    biorbd::rigidbody::Segment DeepCopyLater;
    DeepCopyLater.DeepCopy(MasterSegment);

    EXPECT_STREQ(MasterSegment.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "NoParent");
    EXPECT_STREQ(ShallowCopyEqual.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");
    ShallowCopy.setParent("MyLovelyParent");
    EXPECT_STREQ(MasterSegment.parent().c_str(), "MyLovelyParent");
    EXPECT_STREQ(ShallowCopy.parent().c_str(), "MyLovelyParent");
    EXPECT_STREQ(ShallowCopyEqual.parent().c_str(), "MyLovelyParent");
    EXPECT_STREQ(DeepCopyNow.parent().c_str(), "NoParent");
    EXPECT_STREQ(DeepCopyLater.parent().c_str(), "NoParent");
}

TEST(Mesh, copy)
{
    biorbd::rigidbody::Mesh MasterMesh;
    MasterMesh.setPath("./MyFile.bioMesh");
    biorbd::rigidbody::Mesh ShallowCopy(MasterMesh);
    biorbd::rigidbody::Mesh DeepCopyNow(MasterMesh.DeepCopy());
    biorbd::rigidbody::Mesh DeepCopyLater;
    DeepCopyLater.DeepCopy(MasterMesh);

    EXPECT_STREQ(MasterMesh.path().relativePath().c_str(), "./MyFile.bioMesh");
    EXPECT_STREQ(ShallowCopy.path().relativePath().c_str(), "./MyFile.bioMesh");
    EXPECT_STREQ(DeepCopyNow.path().relativePath().c_str(), "./MyFile.bioMesh");
    EXPECT_STREQ(DeepCopyLater.path().relativePath().c_str(), "./MyFile.bioMesh");
    ShallowCopy.setPath("./MyNewFile.bioMesh");
    EXPECT_STREQ(MasterMesh.path().relativePath().c_str(), "./MyNewFile.bioMesh");
    EXPECT_STREQ(ShallowCopy.path().relativePath().c_str(), "./MyNewFile.bioMesh");
    EXPECT_STREQ(DeepCopyNow.path().relativePath().c_str(), "./MyFile.bioMesh");
    EXPECT_STREQ(DeepCopyLater.path().relativePath().c_str(), "./MyFile.bioMesh");
}

static std::vector<std::vector<double>> expectedMarkers = {
    std::vector<double>({1.0126678074548392, 0.46575286691125295, -0.082379586527044829}),
    std::vector<double>({-0.18232123669751762,  0.98685937986570527,  0.46575286691125295}),
    std::vector<double>({0.39552020666133958, -0.18232123669751762,   1.0126678074548392}),
    std::vector<double>({1.0258667774186612, 1.0702910100794407, 1.1960410878390473})
};
TEST(Markers, allPositions)
{
    biorbd::Model model(modelPathMeshEqualsMarker);
    EXPECT_EQ(model.nbQ(), 6);
    EXPECT_EQ(model.nbMarkers(), 4);

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    for (unsigned int i=0; i<model.nbQ(); ++i) {
        Q[i] = QtestEqualsMarker[i];
    }

    // All markers at once
    std::vector<biorbd::rigidbody::NodeSegment> markers(model.markers(Q, true,
            true));
    for (unsigned int i=0; i<model.nbMarkers(); ++i) {
        for (unsigned int j=0; j<3; ++j) {
            SCALAR_TO_DOUBLE(mark, markers[i][j]);
            EXPECT_NEAR(mark, expectedMarkers[i][j], requiredPrecision);
        }
    }
}

TEST(Markers, individualPositions)
{
    biorbd::Model model(modelPathMeshEqualsMarker);
    EXPECT_EQ(model.nbQ(), 6);
    EXPECT_EQ(model.nbMarkers(), 4);

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    for (unsigned int i=0; i<model.nbQ(); ++i) {
        Q[i] = QtestEqualsMarker[i];
    }

    // One marker at a time, only update Q once
    for (unsigned int i=0; i<model.nbMarkers(); ++i) {
        biorbd::rigidbody::NodeSegment marker;
        if (i==0) {
            marker = model.marker(Q, i, true, true);
        } else {
            marker = model.marker(Q, i, true, false);
        }
        for (unsigned int j=0; j<3; ++j) {
            SCALAR_TO_DOUBLE(mark, marker[j]);
            EXPECT_NEAR(mark, expectedMarkers[i][j], requiredPrecision);
        }
    }

    // Change Q
    FILL_VECTOR(Q, std::vector<double>({0.3, 0.3, 0.3, 0.1, 0.1, 0.1}));
    std::vector<std::vector<double>> expectedMarkers2 = {
        std::vector<double>({1.290033288920621, 0.40925158443563553, 0.21112830525233722}),
        std::vector<double>({0.20066533460246938,  1.2890382781008347, 0.40925158443563558}),
        std::vector<double>({0.39983341664682814, 0.20066533460246938,  1.290033288920621}),
        std::vector<double>({1.2905320401699185, 1.2989551971389397, 1.310413178608594})
    };
    // One marker at a time, only update Q once
    for (unsigned int i=0; i<model.nbMarkers(); ++i) {
        biorbd::rigidbody::NodeSegment marker;
        if (i==0) {
            marker = model.marker(Q, i, true, true);
        } else {
            marker = model.marker(Q, i, true, false);
        }
        for (unsigned int j=0; j<3; ++j) {
            SCALAR_TO_DOUBLE(mark, marker[j]);
            EXPECT_NEAR(mark, expectedMarkers2[i][j], requiredPrecision);
        }
    }
}

TEST(Mesh, position)
{
    biorbd::Model model(modelPathMeshEqualsMarker);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    for (unsigned int q=0; q<model.nbQ(); ++q) {
        Q.setZero();
        Q[q] = 1;
        std::vector<std::vector<biorbd::utils::Vector3d>> mesh(model.meshPoints(Q));
        std::vector<biorbd::rigidbody::NodeSegment> markers(model.markers(Q));
        for (unsigned int idx=0; idx<markers.size(); ++idx) {
            for (unsigned int xyz =0; xyz<3; ++xyz) {
                SCALAR_TO_DOUBLE(meshDouble, mesh[0][idx][xyz]);
                SCALAR_TO_DOUBLE(markerDouble, markers[idx][xyz]);
                EXPECT_NEAR(meshDouble, markerDouble, requiredPrecision);
            }
        }
    }
    {
        Q.setOnes();
        std::vector<std::vector<biorbd::utils::Vector3d>> mesh(model.meshPoints(Q));
        std::vector<biorbd::rigidbody::NodeSegment> markers(model.markers(Q));
        for (unsigned int idx=0; idx<markers.size(); ++idx) {
            for (unsigned int xyz =0; xyz<3; ++xyz) {
                SCALAR_TO_DOUBLE(meshDouble, mesh[0][idx][xyz]);
                SCALAR_TO_DOUBLE(markerDouble, markers[idx][xyz]);
                EXPECT_NEAR(meshDouble, markerDouble, requiredPrecision);
            }
        }
    }
}

TEST(Dynamics, Forward)
{
    biorbd::Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_GENERALIZED_TORQUE(Tau, model);

    // Set to random values
    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); ++i) {
        val[i] = static_cast<double>(i) * 1.1;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    FILL_VECTOR(Tau, val);

    std::vector<double> QDDot_expected = {
        20.554883896960259, -22.317642013324736, -77.406439058256126, 17.382961188212313,
        -63.426361095191858, 93.816468824985876, 106.46105024484631, 95.116641811710167,
        -268.1961283528546, 2680.3632159799949, -183.4582596257801, 755.89411812405604,
        163.60239754283589
    };

    CALL_BIORBD_FUNCTION_3ARGS(QDDot, model, ForwardDynamics, Q, QDot, Tau);

    for (unsigned int i = 0; i<model.nbQddot(); ++i) {
        EXPECT_NEAR(static_cast<double>(QDDot(i, 0)), QDDot_expected[i],
                    requiredPrecision);
    }
}

TEST(Dynamics, ForwardChangingMass)
{
    biorbd::Model model(modelSimple);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_GENERALIZED_TORQUE(Tau, model);

    // Set to random values
    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); ++i) {
        val[i] = static_cast<double>(i) * 1.1;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    FILL_VECTOR(Tau, val);

    biorbd::rigidbody::SegmentCharacteristics c(model.segment(0).characteristics());
    c.setMass(10);
    model.updateSegmentCharacteristics(0, c);

    std::vector<double> QDDot_expected = {0.0, -9.7, 2.2};

    CALL_BIORBD_FUNCTION_3ARGS(QDDot, model, ForwardDynamics, Q, QDot, Tau);

    for (unsigned int i = 0; i<model.nbQddot(); ++i) {
        EXPECT_NEAR(static_cast<double>(QDDot(i, 0)), QDDot_expected[i],
                    requiredPrecision);
    }


}

TEST(Dynamics, ForwardDynAndExternalForces)
{
    biorbd::Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_GENERALIZED_TORQUE(Tau, model);
    std::vector<biorbd::utils::SpatialVector> f_ext;
    for (size_t i=0; i<2; ++i) {
        double di = static_cast<double>(i);
        f_ext.push_back(biorbd::utils::SpatialVector(
                            (di+1)*11.1, (di+1)*22.2, (di+1)*33.3, (di+1)*44.4, (di+1)*55.5, (di+1)*66.6));
    }

    // Set to random values
    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); ++i) {
        val[i] = static_cast<double>(i) * 1.1;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    FILL_VECTOR(Tau, val);

    std::vector<double> QDDot_expected = {
        8.8871711208009998, -13.647827029817943, -33.606145294752132, 16.922669487341341,
        -21.882821189868423, 41.15364990805439, 68.892537246574463, -324.59756885799197,
        -447.99217990207387, 18884.241415786601, -331.24622725851572, 1364.7620674666462,
        3948.4748602722384
    };

    CALL_BIORBD_FUNCTION_3ARGS1PARAM(QDDot, model, ForwardDynamics, Q, QDot, Tau,
                                     &f_ext);

    for (unsigned int i = 0; i<model.nbQddot(); ++i) {
        EXPECT_NEAR(static_cast<double>(QDDot(i, 0)), QDDot_expected[i],
                    requiredPrecision);
    }
}


TEST(QDot, ComputeConstraintImpulsesDirect)
{
    biorbd::Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);

    // Set to random values
    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); ++i) {
        val[i] = static_cast<double>(i) * 1.1;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);

    CALL_BIORBD_FUNCTION_2ARGS(QDotPost, model, ComputeConstraintImpulsesDirect, Q,
                               QDot);

    std::vector<double> QDotPost_expected = {
        0.92034698076739008, 0.4542331948818259, -1.1747551666658667, 3.3396871279100031,
        1.1143307751232683, 9.5534681791265204, 9.5313390358865036, 2.5590424787426884,
        -3.0502066043856577, 1.6659192923088271, 1.3562999563073794, -3.4457346325708458,
        3.2641898429292815
    };
    for (unsigned int i = 0; i<model.nbQdot(); ++i) {
        EXPECT_NEAR(static_cast<double>(QDotPost(i, 0)), QDotPost_expected[i],
                    requiredPrecision);
    }
}


TEST(Dynamics, ForwardLoopConstraint)
{
    biorbd::Model model(modelPathForLoopConstraintTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_GENERALIZED_TORQUE(Tau, model);

    // Set to random values
    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); ++i) {
        val[i] = static_cast<double>(i) * 1.1;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    FILL_VECTOR(Tau, val);

    std::vector<double> QDDot_expected = {
        4357.563983223662,  -1980.272417081602, -4132.170113875329, 34854.96630091612,
        -5939.1875609623385, 20005.793234188295, -33019.84433234081, 5044.593964065896,
        76960.9024224599, 13949749.797541305, 29056.19402773685, 13957133.384121455
    };

    CALL_BIORBD_FUNCTION_3ARGS(QDDot, model, ForwardDynamicsConstraintsDirect, Q,
                               QDot, Tau);
    for (unsigned int i = 0; i<model.nbQddot(); ++i) {
        EXPECT_NEAR(static_cast<double>(QDDot(i, 0)), QDDot_expected[i], 1e-2);
    }
}

TEST(Dynamics, ForwardAccelerationConstraint)
{
    biorbd::Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_GENERALIZED_ACCELERATION(QDDot_constrained, model);
    DECLARE_GENERALIZED_TORQUE(Tau, model);

    std::vector<double> val(model.nbQ());
    for (size_t i=0; i<val.size(); ++i) {
        val[i] = 1.0;
    }
    FILL_VECTOR(Q, val);
    FILL_VECTOR(QDot, val);
    FILL_VECTOR(Tau, val);

    std::vector<double> QDDot_expected = {1.9402069774422919,  -9.1992692111538243,  2.9930159570454702,
                                          5.2738378853554133, 8.9387539396273699, 6.0938738229550751, 9.9560407885164217,
                                          38.6297746304162, -52.159023390563554, 36.702385054876714, 38.629774630416208, -52.159023390563561,
                                          36.70238505487675
                                         };
    std::vector<double> forces_expected = {-16.344680827308579, -30.485214214095951, 112.8234134576031, -16.344680827308611,
                                           -30.485214214095965, 112.82341345760311
                                          };

    biorbd::rigidbody::Contacts cs(model.getConstraints());
    CALL_BIORBD_FUNCTION_3ARGS1PARAM(QDDot, model, ForwardDynamicsConstraintsDirect,
                                     Q, QDot, Tau, cs);
    for (unsigned int i = 0; i<model.nbQddot(); ++i) {
        EXPECT_NEAR(static_cast<double>(QDDot(i, 0)), QDDot_expected[i],
                    requiredPrecision);
    }
    EXPECT_EQ(cs.nbContacts(), forces_expected.size());

    CALL_BIORBD_FUNCTION_3ARGS(forces, model,
                               ContactForcesFromForwardDynamicsConstraintsDirect, Q, QDot, Tau);
    for (unsigned int i=0; i<forces_expected.size(); ++i) {
        EXPECT_NEAR(static_cast<double>(forces(i, 0)), forces_expected[i],
                    requiredPrecision);
    }
}

TEST(QuaternionInModel, sizes)
{
    biorbd::Model m("models/simple_quat.bioMod");
    EXPECT_EQ(m.nbQ(), 4);
    EXPECT_EQ(m.nbQdot(), 3);
    EXPECT_EQ(m.nbQddot(), 3);
    EXPECT_EQ(m.nbGeneralizedTorque(), 3);
}

TEST(Kinematics, computeQdot)
{
    biorbd::Model m("models/simple_quat.bioMod");
    DECLARE_GENERALIZED_VELOCITY(QDot, m);
    FILL_VECTOR(QDot, std::vector<double>({1, 2, 3}));
    {
        DECLARE_GENERALIZED_COORDINATES(Q_quat, m);
        FILL_VECTOR(Q_quat, std::vector<double>({0, 0, 0, 1}));
        std::vector<double> QDot_quat_expected = {0.5, 1, 1.5, 0};

        CALL_BIORBD_FUNCTION_2ARGS(QDot_quat, m, computeQdot, Q_quat, QDot);
        for (unsigned int i=0; i<m.nbQ(); ++i) {
            EXPECT_NEAR(static_cast<double>(QDot_quat(i, 0)), QDot_quat_expected[i],
                        requiredPrecision);
        }
    }
    {
        double w(0.07035975447302918);
        double x(0.7035975447302919);
        double y(0.7035975447302919);
        double z(0.07035975447302918);
        DECLARE_GENERALIZED_COORDINATES(Q_quat, m);
        FILL_VECTOR(Q_quat, std::vector<double>({x, y, z, w}));
        std::vector<double> QDot_quat_expected = {1.0202164398589233, -0.9498566853858941,
                                                  0.45733840407468973,-1.1609359488049815
                                                 };
        CALL_BIORBD_FUNCTION_2ARGS(QDot_quat, m, computeQdot, Q_quat, QDot);

        for (unsigned int i=0; i<m.nbQ(); ++i) {
            EXPECT_NEAR(static_cast<double>(QDot_quat(i, 0)), QDot_quat_expected[i],
                        requiredPrecision);
        }
    }
}

#ifdef MODULE_KALMAN
#ifndef SKIP_LONG_TESTS
TEST(Kalman, markers)
{
    biorbd::Model model(modelPathForGeneralTesting);
    biorbd::rigidbody::KalmanReconsMarkers kalman(model);
#ifdef BIORBD_USE_CASADI_MATH
    // Because the evaluation functions are not sym, it takes a LOOOONG time
    unsigned int nQToTest(1);
#else
    unsigned int nQToTest(model.nbQ());
#endif

    // Compute reference
    biorbd::rigidbody::GeneralizedCoordinates Qref(model);
    for (unsigned int i=0; i<model.nbQ(); ++i) {
        Qref(i, 0) = 0.2;
    }
    std::vector<biorbd::rigidbody::NodeSegment> targetMarkers(model.markers(Qref));

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    biorbd::rigidbody::GeneralizedAcceleration Qddot(model);
    kalman.reconstructFrame(model, targetMarkers, &Q, &Qdot, &Qddot);

    // Compare results (since the initialization of the filter is done 50X, it is expected to have converged)
    for (unsigned int i=0; i<nQToTest; ++i) {
        SCALAR_TO_DOUBLE(q, Q[i]);
        SCALAR_TO_DOUBLE(qdot, Qdot[i]);
        SCALAR_TO_DOUBLE(qddot, Qddot[i]);
        SCALAR_TO_DOUBLE(qref, Qref[i]);
        EXPECT_NEAR(q, qref, 1e-6);
        EXPECT_NEAR(qdot, 0, 1e-6);
        EXPECT_NEAR(qddot, 0, 1e-6);
    }

    for (unsigned int i=0; i<model.nbQ(); ++i) {
        Qref(i, 0) = 0.3;
    }
    targetMarkers = model.markers(Qref);
    kalman.reconstructFrame(model, targetMarkers, &Q, &Qdot, &Qddot);

    // Compare results (Here the filter should not have the time to converge)
    for (unsigned int i=0; i<nQToTest; ++i) {
        SCALAR_TO_DOUBLE(q, Q[i]);
        SCALAR_TO_DOUBLE(qdot, Qdot[i]);
        SCALAR_TO_DOUBLE(qddot, Qddot[i]);
        SCALAR_TO_DOUBLE(qref, Qref[i]);
        EXPECT_GT(fabs(q - qref), 1e-4);
        EXPECT_GT(fabs(qdot), 5);
        EXPECT_GT(fabs(qddot), 100);
    }

    // Force the filter to converge
    for (unsigned int i=0; i<100; ++i) {
        kalman.reconstructFrame(model, targetMarkers, &Q, &Qdot, &Qddot);
    }

    // Now it should be more or less equal
    for (unsigned int i=0; i<nQToTest; ++i) {
        SCALAR_TO_DOUBLE(q, Q[i]);
        SCALAR_TO_DOUBLE(qdot, Qdot[i]);
        SCALAR_TO_DOUBLE(qddot, Qddot[i]);
        SCALAR_TO_DOUBLE(qref, Qref[i]);
        EXPECT_NEAR(q, qref, 1e-6);
        EXPECT_NEAR(qdot, 0, 1e-6);
        EXPECT_NEAR(qddot, 0, 1e-6);
    }
}
#endif

#ifndef SKIP_LONG_TESTS
TEST(Kalman, imu)
{
    biorbd::Model model(modelPathForPyomecaman_withIMUs);
    biorbd::rigidbody::KalmanReconsIMU kalman(model);
#ifdef BIORBD_USE_CASADI_MATH
    // Because the evaluation functions are not sym, it takes a LOOOONG time
    unsigned int nQToTest(3);

    // Test is known to fail for Kalman IMU (TODO: solve this)
    return;
#else
    unsigned int nQToTest(model.nbQ());
#endif

    // Compute reference
    biorbd::rigidbody::GeneralizedCoordinates Qref(model);
    for (unsigned int i=0; i<model.nbQ(); ++i) {
        Qref(i, 0) = 0.2;
    }
    std::vector<biorbd::rigidbody::IMU> targetImus(model.IMU(Qref));

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    biorbd::rigidbody::GeneralizedAcceleration Qddot(model);
    kalman.reconstructFrame(model, targetImus, &Q, &Qdot, &Qddot);

    // Compare results (since the initialization of the filter is done 50X, it is expected to have converged)
    for (unsigned int i=0; i<nQToTest; ++i) {
        SCALAR_TO_DOUBLE(q, Q[i]);
        SCALAR_TO_DOUBLE(qdot, Qdot[i]);
        SCALAR_TO_DOUBLE(qddot, Qddot[i]);
        SCALAR_TO_DOUBLE(qref, Qref[i]);
        if (i < 2) {
            // Translations are not reconstructed from IMU
            EXPECT_EQ(q, 0);
            EXPECT_EQ(qdot, 0);
            EXPECT_EQ(qddot, 0);
        } else {
            EXPECT_NEAR(q, qref, 1e-6);
            EXPECT_NEAR(qdot, 0, 1e-6);
            EXPECT_NEAR(qddot, 0, 1e-6);
        }
    }

    for (unsigned int i=0; i<model.nbQ(); ++i) {
        Qref(i, 0) = 0.3;
    }
    targetImus = model.IMU(Qref);
    kalman.reconstructFrame(model, targetImus, &Q, &Qdot, &Qddot);

    // Compare results (Here the filter should not have the time to converge)
    for (unsigned int i=0; i<nQToTest; ++i) {
        SCALAR_TO_DOUBLE(q, Q[i]);
        SCALAR_TO_DOUBLE(qdot, Qdot[i]);
        SCALAR_TO_DOUBLE(qddot, Qddot[i]);
        SCALAR_TO_DOUBLE(qref, Qref[i]);
        if (i<2) {
            EXPECT_EQ(q, 0);
            EXPECT_EQ(qdot, 0);
            EXPECT_EQ(qddot, 0);
        } else {
            EXPECT_GT(abs(q - qref), 1e-4);
            EXPECT_GT(abs(qdot), 1e-2);
            EXPECT_GT(abs(qddot), 1e-1);
        }
    }

    // Force the filter to converge
    for (unsigned int i=0; i<1000; ++i) {
        kalman.reconstructFrame(model, targetImus, &Q, &Qdot, &Qddot);
    }

    // Now it should be more or less equal
    for (unsigned int i=0; i<model.nbQ(); ++i) {
        SCALAR_TO_DOUBLE(q, Q[i]);
        SCALAR_TO_DOUBLE(qdot, Qdot[i]);
        SCALAR_TO_DOUBLE(qddot, Qddot[i]);
        SCALAR_TO_DOUBLE(qref, Qref[i]);
        if (i < 2) {
            // Translations are not reconstructed from IMU
            EXPECT_EQ(q, 0);
            EXPECT_EQ(qdot, 0);
            EXPECT_EQ(qddot, 0);
        } else {
            EXPECT_NEAR(q, qref, 1e-6);
            EXPECT_NEAR(qdot, 0, 1e-6);
            EXPECT_NEAR(qddot, 0, 1e-6);
        }
    }
}
#endif


#endif
