#include <iostream>
#include <gtest/gtest.h>
#include <rbdl/rbdl_math.h>
#include <rbdl/Dynamics.h>


#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedTorque.h"
#include "Actuators/ActuatorConstant.h"
#include "Actuators/ActuatorGauss3p.h"
#include "Actuators/ActuatorGauss6p.h"
#include "Actuators/ActuatorLinear.h"
#include "Actuators/ActuatorSigmoidGauss3p.h"

static std::string
modelPathForGeneralTesting("models/pyomecaman_withActuators.bioMod");
static std::string
modelPathWithMissingActuator("models/withMissingActuator.bioMod");
static std::string modelPathWithoutActuator("models/pyomecaman.bioMod");
static std::string
modelPathWithAllActuators("models/withAllActuatorsTypes.bioMod");


static double requiredPrecision(1e-10);

TEST(FileIO, openModelWithActuators)
{
    EXPECT_NO_THROW(biorbd::Model model(modelPathForGeneralTesting));
    EXPECT_NO_THROW(biorbd::Model model(modelPathWithoutActuator));
    EXPECT_THROW(biorbd::Model model(modelPathWithMissingActuator),
                 std::runtime_error);
    EXPECT_NO_THROW(biorbd::Model model(modelPathWithAllActuators));
}

TEST(ActuatorConstant, torqueMax)
{
    biorbd::actuator::ActuatorConstant const_torque_act(1, 150, 0);
    SCALAR_TO_DOUBLE(torqueMaxVal, const_torque_act.torqueMax())
    EXPECT_NEAR(torqueMaxVal, 150, requiredPrecision);
}

TEST(ActuatorGauss3p, torqueMax)
{
    // A model is loaded so Q can be > 0 in size, it is not used otherwise
    biorbd::Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);

    biorbd::actuator::ActuatorGauss3p gauss3p_torque_act(1, 150, 25, 800, 324, 0.5,
            28, 90, 29, 133, 0);
    std::vector<double> Q_val = {1.1, 1.1, 1.1, 1.1, 1.1};
    FILL_VECTOR(Q, Q_val);
    {
        std::vector<double> QDot_val = {10, 10, 10, 10, 10};
        double torqueMaxExpected(0.13946332238760348);
        FILL_VECTOR(QDot, QDot_val);
        CALL_BIORBD_FUNCTION_2ARGS(torqueMaxVal, gauss3p_torque_act, torqueMax, Q,
                                   QDot);
#ifdef BIORBD_USE_CASADI_MATH
        EXPECT_NEAR(static_cast<double>(torqueMaxVal(0, 0)), torqueMaxExpected,
                    requiredPrecision);
#else
        EXPECT_NEAR(torqueMaxVal, torqueMaxExpected, requiredPrecision);
#endif
    }
    {
        std::vector<double> QDot_val = {-10, -10, -10, -10, -10};
        double torqueMaxExpected(2.9969806062215922);
        FILL_VECTOR(QDot, QDot_val);
        CALL_BIORBD_FUNCTION_2ARGS(torqueMaxVal, gauss3p_torque_act, torqueMax, Q,
                                   QDot);
#ifdef BIORBD_USE_CASADI_MATH
        EXPECT_NEAR(static_cast<double>(torqueMaxVal(0, 0)), torqueMaxExpected,
                    requiredPrecision);
#else
        EXPECT_NEAR(torqueMaxVal, torqueMaxExpected, requiredPrecision);
#endif
    }
}

TEST(ActuatorGauss6p, torqueMax)
{
    // A model is loaded so Q can be > 0 in size, it is not used otherwise
    biorbd::Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);

    biorbd::actuator::ActuatorGauss6p gauss6p_torque_act(1, 150, 25, 800, 324, 0.5,
            28, 90, 29, 133, 4, 73, 73, 0);
    std::vector<double> Q_val = {1.1, 1.1, 1.1, 1.1, 1.1};
    FILL_VECTOR(Q, Q_val);
    {
        std::vector<double> QDot_val = {10, 10, 10, 10, 10};
        double torqueMaxExpected(10.295760991374534);
        FILL_VECTOR(QDot, QDot_val);
        CALL_BIORBD_FUNCTION_2ARGS(torqueMaxVal, gauss6p_torque_act, torqueMax, Q,
                                   QDot);
#ifdef BIORBD_USE_CASADI_MATH
        EXPECT_NEAR(static_cast<double>(torqueMaxVal(0, 0)), torqueMaxExpected,
                    requiredPrecision);
#else
        EXPECT_NEAR(torqueMaxVal, torqueMaxExpected, requiredPrecision);
#endif
    }
    {
        std::vector<double> QDot_val = {-10, -10, -10, -10, -10};
        double torqueMaxExpected(221.2495406619181);
        FILL_VECTOR(QDot, QDot_val);
        CALL_BIORBD_FUNCTION_2ARGS(torqueMaxVal, gauss6p_torque_act, torqueMax, Q,
                                   QDot);
#ifdef BIORBD_USE_CASADI_MATH
        EXPECT_NEAR(static_cast<double>(torqueMaxVal(0, 0)), torqueMaxExpected,
                    requiredPrecision);
#else
        EXPECT_NEAR(torqueMaxVal, torqueMaxExpected, requiredPrecision);
#endif
    }
}

TEST(ActuatorLinear, torqueMax)
{
    // A model is loaded so Q can be > 0 in size, it is not used otherwise
    biorbd::Model model(modelPathForGeneralTesting);
    DECLARE_GENERALIZED_COORDINATES(Q, model);

    std::vector<double> val = {1.1};
    FILL_VECTOR(Q, val);
    double torqueMaxExpected(88.025357464390567);
    biorbd::actuator::ActuatorLinear linear_torque_act(1, 25, 1, 0);
    CALL_BIORBD_FUNCTION_1ARG(torqueMaxVal, linear_torque_act, torqueMax, Q);
#ifdef BIORBD_USE_CASADI_MATH
    EXPECT_NEAR(static_cast<double>(torqueMaxVal(0, 0)), torqueMaxExpected,
                requiredPrecision);
#else
    EXPECT_NEAR(torqueMaxVal, torqueMaxExpected, requiredPrecision);
#endif
}

TEST(Actuators, NbActuators)
{
    {
        biorbd::Model model(modelPathForGeneralTesting);
        size_t val(model.nbActuators());
        EXPECT_NEAR(val, 13, requiredPrecision);
    }
    {
        biorbd::Model model(modelPathWithoutActuator);
        size_t val(model.nbActuators());
        EXPECT_NEAR(val, 0, requiredPrecision);
    }
}

TEST(Actuators, jointTorqueFromAllTypesOfActuators)
{
    biorbd::Model model(modelPathWithAllActuators);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);
    DECLARE_VECTOR(actuatorActivations, model.nbGeneralizedTorque());
    std::vector<double> Q_val(model.nbQ());
    for (size_t i=0; i<Q_val.size(); ++i) {
        Q_val[i] = 1.1;
    }
    FILL_VECTOR(Q, Q_val);
    {
        std::vector<double> QDot_val(model.nbQdot());
        for (size_t i=0; i<QDot_val.size(); ++i) {
            QDot_val[i] = 1.1;
        }
        FILL_VECTOR(QDot, QDot_val);

        {
            std::vector<double> act_val(model.nbGeneralizedTorque());
            for (size_t i=0; i<act_val.size(); ++i) {
                act_val[i] = 0.5;
            }
            FILL_VECTOR(actuatorActivations, act_val);

            std::vector<double> tauExpected = {5, 45.35073139870569, 24.688715372701648, 162.56339366097643, 16.208968606473185};
            std::vector<double> torqueMaxExpected = {10, 90.701462797411381, 49.377430745403295, 325.12678732195286, 32.41793721294637};

            CALL_BIORBD_FUNCTION_3ARGS(tau, model, torque, actuatorActivations, Q, QDot);
#ifdef BIORBD_USE_CASADI_MATH
            EXPECT_NEAR(tau.size().first, 5, requiredPrecision);
#else
            EXPECT_NEAR(tau.size(), 5, requiredPrecision);
#endif
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i) {
                EXPECT_NEAR(static_cast<double>(tau(i, 0)), tauExpected[i], requiredPrecision);
            }
            CALL_BIORBD_FUNCTION_3ARGS(torqueMax, model, torqueMax, actuatorActivations, Q,
                                       QDot);
#ifdef BIORBD_USE_CASADI_MATH
            EXPECT_NEAR(torqueMax.size().first, 5, requiredPrecision);
#else
            EXPECT_NEAR(torqueMax.size(), 5, requiredPrecision);
#endif
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i) {
                EXPECT_NEAR(static_cast<double>(torqueMax(i, 0)), torqueMaxExpected[i],
                            requiredPrecision);
            }
        }
        {
            std::vector<double> act_val(model.nbGeneralizedTorque());
            for (size_t i=0; i<act_val.size(); ++i) {
                act_val[i] = -0.5;
            }

            FILL_VECTOR(actuatorActivations, act_val);

            std::vector<double> tauExpected = {-2.5, -71.180973314287982, -28.111094840325926, -160.06339366097643, -30.025156051078717};
            std::vector<double> torqueMaxExpected = {5, 142.36194662857596, 56.222189680651852, 320.12678732195286, 60.050312102157434};

            CALL_BIORBD_FUNCTION_3ARGS(tau, model, torque, actuatorActivations, Q, QDot);
#ifdef BIORBD_USE_CASADI_MATH
            EXPECT_NEAR(tau.size().first, 5, requiredPrecision);
#else
            EXPECT_NEAR(tau.size(), 5, requiredPrecision);
#endif
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i) {
                EXPECT_NEAR(static_cast<double>(tau(i, 0)), tauExpected[i], requiredPrecision);
            }
            CALL_BIORBD_FUNCTION_3ARGS(torqueMax, model, torqueMax, actuatorActivations, Q,
                                       QDot);
#ifdef BIORBD_USE_CASADI_MATH
            EXPECT_NEAR(torqueMax.size().first, 5, requiredPrecision);
#else
            EXPECT_NEAR(torqueMax.size(), 5, requiredPrecision);
#endif
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i) {
                EXPECT_NEAR(static_cast<double>(torqueMax(i, 0)), torqueMaxExpected[i],
                            requiredPrecision);
            }
        }

    }
    {
        std::vector<double> QDot_val(model.nbQdot());
        for (size_t i=0; i<QDot_val.size(); ++i) {
            QDot_val[i] = -1.1;
        }
        FILL_VECTOR(QDot, QDot_val);
        {
            std::vector<double> act_val(model.nbGeneralizedTorque());
            for (size_t i=0; i<act_val.size(); ++i) {
                act_val[i] = 0.5;
            }

            FILL_VECTOR(actuatorActivations, act_val);

            std::vector<double> tauExpected = {5, 85.742784548430208, 30.545039941111146, 162.56339366097643, 30.025156051078717};
            std::vector<double> torqueMaxExpected = {10, 171.48556909686042, 61.090079882222291, 325.12678732195286, 60.050312102157434};

            CALL_BIORBD_FUNCTION_3ARGS(tau, model, torque, actuatorActivations, Q, QDot);
#ifdef BIORBD_USE_CASADI_MATH
            EXPECT_NEAR(tau.size().first, 5, requiredPrecision);
#else
            EXPECT_NEAR(tau.size(), 5, requiredPrecision);
#endif
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i) {
                EXPECT_NEAR(static_cast<double>(tau(i, 0)), tauExpected[i], requiredPrecision);
            }
            CALL_BIORBD_FUNCTION_3ARGS(torqueMax, model, torqueMax, actuatorActivations, Q,
                                       QDot);
#ifdef BIORBD_USE_CASADI_MATH
            EXPECT_NEAR(torqueMax.size().first, 5, requiredPrecision);
#else
            EXPECT_NEAR(torqueMax.size(), 5, requiredPrecision);
#endif
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i) {
                EXPECT_NEAR(static_cast<double>(torqueMax(i, 0)), torqueMaxExpected[i],
                            requiredPrecision);
            }
        }
        {
            std::vector<double> act_val(model.nbGeneralizedTorque());
            for (size_t i=0; i<act_val.size(); ++i) {
                act_val[i] = -0.5;
            }
            FILL_VECTOR(actuatorActivations, act_val);

            std::vector<double> tauExpected = {-2.5, -49.674818618770644, -28.953932124045757, -160.06339366097643, -16.208968606473185};
            std::vector<double> torqueMaxExpected = {5, 99.349637237541288, 57.907864248091514, 320.12678732195286, 32.41793721294637};

            CALL_BIORBD_FUNCTION_3ARGS(tau, model, torque, actuatorActivations, Q, QDot);
#ifdef BIORBD_USE_CASADI_MATH
            EXPECT_NEAR(tau.size().first, 5, requiredPrecision);
#else
            EXPECT_NEAR(tau.size(), 5, requiredPrecision);
#endif
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i) {
                EXPECT_NEAR(static_cast<double>(tau(i, 0)), tauExpected[i], requiredPrecision);
            }
            CALL_BIORBD_FUNCTION_3ARGS(torqueMax, model, torqueMax, actuatorActivations, Q,
                                       QDot);
#ifdef BIORBD_USE_CASADI_MATH
            EXPECT_NEAR(torqueMax.size().first, 5, requiredPrecision);
#else
            EXPECT_NEAR(torqueMax.size(), 5, requiredPrecision);
#endif
            for (size_t i=0; i<model.nbGeneralizedTorque(); ++i) {
                EXPECT_NEAR(static_cast<double>(torqueMax(i, 0)), torqueMaxExpected[i],
                            requiredPrecision);
            }
        }
    }
}

TEST(ActuatorSigmoidGauss3p, torqueMax)
{
    // A model is loaded so Q can be > 0 in size, it is not used otherwise
    biorbd::Model model(modelPathWithAllActuators);
    DECLARE_GENERALIZED_COORDINATES(Q, model);
    DECLARE_GENERALIZED_VELOCITY(QDot, model);

    biorbd::actuator::ActuatorSigmoidGauss3p sigmoid_gauss3p_torque_act(1,
            312.0780851217, 0.0100157340, 3.2702903919,
            56.4021127893, -25.6939435543, 0);
    std::vector<double> Q_val(model.nbQ());
    for (size_t i=0; i<Q_val.size(); ++i) {
        Q_val[i] = 1.1;
    }
    FILL_VECTOR(Q, Q_val);
    {
        std::vector<double> QDot_val(model.nbQdot());
        for (size_t i=0; i<QDot_val.size(); ++i) {
            QDot_val[i] = 10;
        }

        double torqueMaxExpected(1.2397259313103695);
        FILL_VECTOR(QDot, QDot_val);
        CALL_BIORBD_FUNCTION_2ARGS(torqueMaxVal, sigmoid_gauss3p_torque_act, torqueMax,
                                   Q, QDot);
#ifdef BIORBD_USE_CASADI_MATH
        EXPECT_NEAR(static_cast<double>(torqueMaxVal(0, 0)), torqueMaxExpected,
                    requiredPrecision);
#else
        EXPECT_NEAR(torqueMaxVal, torqueMaxExpected, requiredPrecision);
#endif
    }
    {
        std::vector<double> QDot_val(model.nbQdot());
        for (size_t i=0; i<QDot_val.size(); ++i) {
            QDot_val[i] = -10;
        }

        double torqueMaxExpected(91.22845304936142);
        FILL_VECTOR(QDot, QDot_val);
        CALL_BIORBD_FUNCTION_2ARGS(torqueMaxVal, sigmoid_gauss3p_torque_act, torqueMax,
                                   Q, QDot);
#ifdef BIORBD_USE_CASADI_MATH
        EXPECT_NEAR(static_cast<double>(torqueMaxVal(0, 0)), torqueMaxExpected,
                    requiredPrecision);
#else
        EXPECT_NEAR(torqueMaxVal, torqueMaxExpected, requiredPrecision);
#endif
    }
}
