#include <iostream>
#include <gtest/gtest.h>

#include <rbdl/Dynamics.h>
#include "BiorbdModel.h"
#include "biorbdConfig.h"
#include "Utils/Matrix.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/NodeSegment.h"
#ifdef MODULE_MUSCLES
#include "Muscles/all.h"
#include "Utils/String.h"
#include "Utils/RotoTrans.h"

static double requiredPrecision(1e-10);

static std::string modelPathForMuscleForce("models/arm26.bioMod");
static std::string modelPathForBuchananDynamics("models/arm26_buchanan.bioMod");
static std::string modelPathForDeGrooteDynamics("models/arm26_degroote.bioMod");
static std::string modelPathForMuscleJacobian("models/arm26.bioMod");
static unsigned int muscleGroupForMuscleJacobian(1);
static unsigned int muscleForMuscleJacobian(1);

static unsigned int muscleGroupForIdealizedActuator(1);
static unsigned int muscleForIdealizedActuator(1);

TEST(Muscles, size)
{
    biorbd::Model model(modelPathForMuscleForce);
    unsigned int nbMus(model.nbMuscles());
    EXPECT_EQ(nbMus, 6);
    EXPECT_EQ(nbMus, model.nbMuscleTotal());
    EXPECT_EQ(model.muscles().size(), nbMus);
    int cmp(0);
    for (auto g : model.muscleGroups()) {
        cmp += g.nbMuscles();
    }
    EXPECT_EQ(nbMus, cmp);

    cmp = 0;
    for (auto g : model.muscleGroups()) {
        for (auto m : g.muscles()) {
            EXPECT_STREQ(m->name().c_str(), model.muscle(cmp).name().c_str());
            EXPECT_STREQ(m->name().c_str(), model.muscles()[cmp]->name().c_str());
            cmp++;
        }
    }

    EXPECT_THROW(model.muscle(cmp).name().c_str(), std::runtime_error);
}

TEST(IdealizedActuator, unitTest)
{
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::IdealizedActuator idealizedActuator(model.
                muscleGroup(muscleGroupForIdealizedActuator).
                muscle(muscleForIdealizedActuator));
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        Q = Q.setOnes() / 10;
        SCALAR_TO_DOUBLE(length, idealizedActuator.length(model, Q, 2));
        SCALAR_TO_DOUBLE(musculoTendonLength,
                         idealizedActuator.musculoTendonLength(model, Q, 2));
        EXPECT_NEAR(length, 0.066381977535807504, requiredPrecision);
        EXPECT_NEAR(musculoTendonLength, 0.1563647052655904, requiredPrecision);
    }

    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::IdealizedActuator idealizedActuator(model.
                muscleGroup(muscleGroupForIdealizedActuator).
                muscle(muscleForIdealizedActuator));
        idealizedActuator.setName("nom");
        EXPECT_STREQ(idealizedActuator.name().c_str(), "nom");
    }

    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        biorbd::rigidbody::GeneralizedVelocity qDot(model);
        Q = Q.setOnes() / 10;
        qDot = qDot.setOnes() / 10;
        biorbd::muscles::IdealizedActuator idealizedActuator(model.
                muscleGroup(muscleGroupForIdealizedActuator).
                muscle(muscleForIdealizedActuator));
        SCALAR_TO_DOUBLE(velocityTrue, idealizedActuator.velocity(model, Q, qDot,
                         true));
        SCALAR_TO_DOUBLE(velocityFalse, idealizedActuator.velocity(model, Q, qDot,
                         false));
        EXPECT_NEAR(velocityTrue, 0.0022230374109936529, requiredPrecision);
        EXPECT_NEAR(velocityFalse, 0.0022230374109936529, requiredPrecision);
    }

    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::IdealizedActuator idealizedActuator(model.
                muscleGroup(muscleGroupForIdealizedActuator).
                muscle(muscleForIdealizedActuator));
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        biorbd::rigidbody::GeneralizedVelocity qDot(model);
        Q = Q.setOnes() / 10;
        qDot = qDot.setOnes() / 10;
        static double activationEmgForHillTypeTest(1);
        biorbd::muscles::StateDynamics emg(0, activationEmgForHillTypeTest);

        SCALAR_TO_DOUBLE(emg1, idealizedActuator.force(emg));
        SCALAR_TO_DOUBLE(emg2, idealizedActuator.force(model, Q, emg));
        SCALAR_TO_DOUBLE(emg3, idealizedActuator.force(model, Q, qDot, emg));
        EXPECT_NEAR(emg1, 624.29999999999995, requiredPrecision);
        EXPECT_NEAR(emg2, 624.29999999999995, requiredPrecision);
        EXPECT_NEAR(emg3, 624.29999999999995, requiredPrecision);
    }

    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::IdealizedActuator idealizedActuatorOrigin(model.
                muscleGroup(muscleGroupForIdealizedActuator).
                muscle(muscleForIdealizedActuator));

        biorbd::muscles::IdealizedActuator idealizedActuatorNew(
            "newName",
            idealizedActuatorOrigin.position(),
            idealizedActuatorOrigin.characteristics());

        EXPECT_STREQ(idealizedActuatorNew.name().c_str(), "newName");
        EXPECT_EQ(idealizedActuatorNew.type(),
                  biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::IdealizedActuator idealizedActuatorOrigin(model.
                muscleGroup(muscleGroupForIdealizedActuator).
                muscle(muscleForIdealizedActuator));

        biorbd::muscles::IdealizedActuator idealizedActuatorNew(
            "newName",
            idealizedActuatorOrigin.position(),
            idealizedActuatorOrigin.characteristics(),
            idealizedActuatorOrigin.state());

        EXPECT_STREQ(idealizedActuatorNew.name().c_str(), "newName");
        EXPECT_EQ(idealizedActuatorNew.type(),
                  biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::IdealizedActuator idealizedActuatorOrigin(model.
                muscleGroup(muscleGroupForIdealizedActuator).
                muscle(muscleForIdealizedActuator));

        biorbd::muscles::IdealizedActuator idealizedActuatorNew(
            "newName",
            idealizedActuatorOrigin.position(),
            idealizedActuatorOrigin.characteristics());

        EXPECT_STREQ(idealizedActuatorNew.name().c_str(), "newName");
        EXPECT_EQ(idealizedActuatorNew.type(),
                  biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::IdealizedActuator idealizedActuatorOrigin(model.
                muscleGroup(muscleGroupForIdealizedActuator).
                muscle(muscleForIdealizedActuator));

        biorbd::muscles::IdealizedActuator idealizedActuatorNew(
            "newName",
            idealizedActuatorOrigin.position(),
            idealizedActuatorOrigin.characteristics(),
            idealizedActuatorOrigin.pathModifier());

        EXPECT_STREQ(idealizedActuatorNew.name().c_str(), "newName");
        EXPECT_EQ(idealizedActuatorNew.type(),
                  biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::IdealizedActuator idealizedActuatorOrigin(model.
                muscleGroup(muscleGroupForIdealizedActuator).
                muscle(muscleForIdealizedActuator));

        biorbd::muscles::IdealizedActuator idealizedActuatorNew(
            "newName",
            idealizedActuatorOrigin.position(),
            idealizedActuatorOrigin.characteristics(),
            idealizedActuatorOrigin.pathModifier(),
            idealizedActuatorOrigin.state());

        EXPECT_STREQ(idealizedActuatorNew.name().c_str(), "newName");
        EXPECT_EQ(idealizedActuatorNew.type(),
                  biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR);

    }
}

TEST(IdealizedActuator, copy)
{
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::IdealizedActuator idealizedActuator(
            model.muscleGroup(muscleGroupForIdealizedActuator).muscle(
                muscleForIdealizedActuator));
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        Q = Q.setOnes() / 10;

        biorbd::muscles::IdealizedActuator shallowCopy(idealizedActuator);
        biorbd::muscles::IdealizedActuator deepCopyNow(idealizedActuator.DeepCopy());
        biorbd::muscles::IdealizedActuator deepCopyLater;
        deepCopyLater.DeepCopy(idealizedActuator);

        biorbd::utils::String originalName(idealizedActuator.name());
        EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());

        biorbd::utils::String newName("MyNewMuscleName");
        idealizedActuator.setName(newName);
        EXPECT_STREQ(idealizedActuator.name().c_str(), newName.c_str());
        EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
    }

    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::IdealizedActuator idealizedActuator(
            model.muscleGroup(muscleGroupForIdealizedActuator).muscle(
                muscleForIdealizedActuator));

        biorbd::muscles::IdealizedActuator shallowCopy(idealizedActuator);
        biorbd::muscles::IdealizedActuator deepCopyNow(idealizedActuator.DeepCopy());
        biorbd::muscles::IdealizedActuator deepCopyLater;
        deepCopyLater.DeepCopy(idealizedActuator);

        {
            SCALAR_TO_DOUBLE(pennationAngle,
                             idealizedActuator.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(shallowCopyPennationAngle,
                             shallowCopy.characteristics().pennationAngle());
            EXPECT_NEAR(pennationAngle, 0.15707963, requiredPrecision);
            EXPECT_NEAR(shallowCopyPennationAngle, 0.15707963, requiredPrecision);
        }

        biorbd::muscles::Characteristics charac(idealizedActuator.characteristics());
        charac.setPennationAngle(0.523599);
        {
            SCALAR_TO_DOUBLE(pennationAngle,
                             idealizedActuator.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(shallowCopyPennationAngle,
                             shallowCopy.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(deepCopyNowPennationAngle,
                             deepCopyNow.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(deepCopyLaterPennationAngle,
                             deepCopyLater.characteristics().pennationAngle());
            EXPECT_NEAR(pennationAngle, 0.523599, requiredPrecision);
            EXPECT_NEAR(shallowCopyPennationAngle, 0.523599, requiredPrecision);
            EXPECT_NEAR(deepCopyNowPennationAngle, 0.15707963, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterPennationAngle, 0.15707963, requiredPrecision);
        }
    }

    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::IdealizedActuator idealizedActuator(
            model.muscleGroup(muscleGroupForIdealizedActuator).muscle(
                muscleForIdealizedActuator));
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        biorbd::rigidbody::GeneralizedVelocity qDot(model);
        Q = Q.setOnes() / 10;
        qDot = qDot.setOnes() / 10;
        idealizedActuator.updateOrientations(model, Q, qDot);


        biorbd::muscles::IdealizedActuator shallowCopy(idealizedActuator);
        biorbd::muscles::IdealizedActuator deepCopyNow(idealizedActuator.DeepCopy());
        biorbd::muscles::IdealizedActuator deepCopyLater;
        deepCopyLater.DeepCopy(idealizedActuator);


        {
            SCALAR_TO_DOUBLE(length, idealizedActuator.position().length());
            SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
            SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
            SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
            EXPECT_NEAR(length, 0.066381977535807504, requiredPrecision);
            EXPECT_NEAR(shallowCopyLength, 0.066381977535807504, requiredPrecision);
            EXPECT_NEAR(deepCopyNowLength, 0.066381977535807504, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterLength, 0.066381977535807504, requiredPrecision);
        }

        {
            // Change the position of the insertion and pennation angle and compare again (length and insertion in Local)
            biorbd::muscles::Characteristics charac(idealizedActuator.characteristics());
            charac.setPennationAngle(0.523599);
            biorbd::utils::Vector3d insertion(
                idealizedActuator.position().insertionInLocal());
            insertion.set(0.2, 0.2, 0.2);
            biorbd::utils::String oldName(insertion.biorbd::utils::Node::name());
            biorbd::utils::String newName("MyNewName");
            insertion.setName(newName);
            idealizedActuator.updateOrientations(model, Q, qDot, 2);

            {
                SCALAR_TO_DOUBLE(length, idealizedActuator.position().length());
                SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
                SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
                SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
                EXPECT_NEAR(length, 0.07570761027741163, requiredPrecision);
                EXPECT_NEAR(shallowCopyLength, 0.07570761027741163, requiredPrecision);
                EXPECT_NEAR(deepCopyNowLength, 0.066381977535807504, requiredPrecision);
                EXPECT_NEAR(deepCopyLaterLength, 0.066381977535807504, requiredPrecision);
                EXPECT_EQ(
                    idealizedActuator.position().insertionInLocal().biorbd::utils::Node::name(),
                    newName);
                EXPECT_EQ(shallowCopy.position().insertionInLocal().biorbd::utils::Node::name(),
                          newName);
                EXPECT_EQ(deepCopyNow.position().insertionInLocal().biorbd::utils::Node::name(),
                          oldName);
                EXPECT_EQ(
                    deepCopyLater.position().insertionInLocal().biorbd::utils::Node::name(),
                    oldName);
            }

        }

        {
            // Change the position giving an actual vector
            biorbd::utils::Vector3d newPosition(1, 2, 3);
            biorbd::utils::String oldName("MyNewName");
            biorbd::utils::String newName("MyNewNewName");
            biorbd::rigidbody::NodeSegment newNode(newPosition, newName, "", true, true, "",
                                                   0);
            {
                const_cast<biorbd::muscles::Geometry&>(idealizedActuator.position()).setOrigin(
                    newPosition);
                const_cast<biorbd::muscles::Geometry&>
                (idealizedActuator.position()).setInsertionInLocal(newPosition);
                const biorbd::utils::Vector3d& origin =
                    idealizedActuator.position().originInLocal();
                const biorbd::utils::Vector3d& insertion =
                    idealizedActuator.position().insertionInLocal();
                EXPECT_STREQ(origin.biorbd::utils::Node::name().c_str(), "TRImed_origin");
                EXPECT_STREQ(insertion.biorbd::utils::Node::name().c_str(), oldName.c_str());
            }
            {
                const_cast<biorbd::muscles::Geometry&>(idealizedActuator.position()).setOrigin(
                    newNode);
                const_cast<biorbd::muscles::Geometry&>
                (idealizedActuator.position()).setInsertionInLocal(newNode);
                const biorbd::utils::Vector3d& origin =
                    idealizedActuator.position().originInLocal();
                const biorbd::utils::Vector3d& insertion =
                    idealizedActuator.position().insertionInLocal();
                EXPECT_STREQ(origin.biorbd::utils::Node::name().c_str(), newName.c_str());
                EXPECT_STREQ(insertion.biorbd::utils::Node::name().c_str(), newName.c_str());
            }
        }
    }

    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::IdealizedActuator idealizedActuator(
            model.muscleGroup(muscleGroupForIdealizedActuator).muscle(
                muscleForIdealizedActuator));

        biorbd::muscles::IdealizedActuator shallowCopy(idealizedActuator);
        biorbd::muscles::IdealizedActuator deepCopyNow(idealizedActuator.DeepCopy());
        biorbd::muscles::IdealizedActuator deepCopyLater;
        deepCopyLater.DeepCopy(idealizedActuator);

        {
            SCALAR_TO_DOUBLE(excitation, idealizedActuator.state().excitation());
            SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyLaterExcitation, deepCopyLater.state().excitation());
            EXPECT_NEAR(excitation, 0., requiredPrecision);
            EXPECT_NEAR(shallowCopyExcitation, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
        }

        idealizedActuator.state().setExcitation(biorbd::utils::Scalar(5.));

        {
            SCALAR_TO_DOUBLE(excitation, idealizedActuator.state().excitation());
            SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyLaterExcitation, deepCopyLater.state().excitation());
            EXPECT_NEAR(excitation, 5., requiredPrecision);
            EXPECT_NEAR(shallowCopyExcitation, 5., requiredPrecision);
            EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
        }
    }
}

static unsigned int muscleGroupForHillType(1);
static unsigned int muscleForHillType(1);

TEST(hillType, unitTest)
{
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillType hillType(
            model.muscleGroup(muscleGroupForHillType).muscle(
                muscleForHillType));
        hillType.setName("newName");
        EXPECT_STREQ(hillType.name().c_str(), "newName");

    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillType hillType(
            model.muscleGroup(muscleGroupForHillType).muscle(
                muscleForHillType));
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        biorbd::rigidbody::GeneralizedVelocity qDot(model);
        Q = Q.setOnes() / 10;
        qDot = qDot.setOnes() / 10;
        hillType.updateOrientations(model, Q, qDot);
        static double activationEmgForHillTypeTest(1);
        biorbd::muscles::StateDynamics emg(0, activationEmgForHillTypeTest);

        SCALAR_TO_DOUBLE(flce, hillType.FlCE(emg));
        SCALAR_TO_DOUBLE(flpe, hillType.FlPE());
        SCALAR_TO_DOUBLE(fvce, hillType.FvCE());
        SCALAR_TO_DOUBLE(damping, hillType.damping());
        SCALAR_TO_DOUBLE(force, hillType.force(emg));
        EXPECT_NEAR(flce, 0.67988981401208015, requiredPrecision);
        EXPECT_NEAR(flpe, 0., requiredPrecision);
        EXPECT_NEAR(fvce, 1.000886825333013, requiredPrecision);
        EXPECT_NEAR(damping, 0.00019534599393617336, requiredPrecision);
        EXPECT_NEAR(force, 424.95358302550062, requiredPrecision);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillType originalHillType(
            model.muscleGroup(muscleGroupForHillType).muscle(
                muscleForHillType));

        biorbd::muscles::HillType newHillType(
            "newName",
            originalHillType.position(),
            originalHillType.characteristics());

        EXPECT_STREQ(newHillType.name().c_str(), "newName");
        EXPECT_EQ(newHillType.type(), biorbd::muscles::MUSCLE_TYPE::HILL);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillType originalHillType(
            model.muscleGroup(muscleGroupForHillType).muscle(
                muscleForHillType));

        biorbd::muscles::HillType newHillType(
            "newName",
            originalHillType.position(),
            originalHillType.characteristics(),
            originalHillType.state());

        EXPECT_STREQ(newHillType.name().c_str(), "newName");
        EXPECT_EQ(newHillType.type(), biorbd::muscles::MUSCLE_TYPE::HILL);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillType originalHillType(
            model.muscleGroup(muscleGroupForHillType).muscle(
                muscleForHillType));

        biorbd::muscles::HillType newHillType(
            "newName",
            originalHillType.position(),
            originalHillType.characteristics(),
            originalHillType.pathModifier());

        EXPECT_STREQ(newHillType.name().c_str(), "newName");
        EXPECT_EQ(newHillType.type(), biorbd::muscles::MUSCLE_TYPE::HILL);

    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillType hillType(
            model.muscleGroup(muscleGroupForHillType).muscle(
                muscleForHillType));
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        biorbd::rigidbody::GeneralizedVelocity qDot(model);
        Q = Q.setOnes() / 10;
        qDot = qDot.setOnes() / 10;
        static double activationEmgForHillTypeTest(1);
        biorbd::muscles::StateDynamics emg(0, activationEmgForHillTypeTest);

        SCALAR_TO_DOUBLE(force, hillType.force(model, Q, qDot, emg, 2));
        EXPECT_NEAR(force, 424.95358302550062, requiredPrecision);
    }
}

TEST(hillType, copy)
{
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillType hillType(
            model.muscleGroup(muscleGroupForHillType).muscle(
                muscleForHillType));
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        Q = Q.setOnes() / 10;

        biorbd::muscles::HillType shallowCopy(hillType);
        biorbd::muscles::HillType deepCopyNow(hillType.DeepCopy());
        biorbd::muscles::HillType deepCopyLater;
        deepCopyLater.DeepCopy(hillType);

        biorbd::utils::String originalName(hillType.name());
        EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());

        biorbd::utils::String newName("MyNewMuscleName");
        hillType.setName(newName);
        EXPECT_STREQ(hillType.name().c_str(), newName.c_str());
        EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillType hillType(
            model.muscleGroup(muscleGroupForHillType).muscle(
                muscleForHillType));

        //copies of the Hill Type muscle
        biorbd::muscles::HillType shallowCopy(hillType);
        biorbd::muscles::HillType deepCopyNow(hillType.DeepCopy());
        biorbd::muscles::HillType deepCopyLater;
        deepCopyLater.DeepCopy(hillType);

        {
            SCALAR_TO_DOUBLE(pennationAngle, hillType.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(shallowCopyPennationAngle,
                             shallowCopy.characteristics().pennationAngle());
            EXPECT_NEAR(pennationAngle, 0.15707963, requiredPrecision);
            EXPECT_NEAR(shallowCopyPennationAngle, 0.15707963, requiredPrecision);
        }

        biorbd::muscles::Characteristics charac(hillType.characteristics());
        charac.setPennationAngle(0.523599);
        {
            SCALAR_TO_DOUBLE(pennationAngle, hillType.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(shallowCopyPennationAngle,
                             shallowCopy.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(deepCopyNowPennationAngle,
                             deepCopyNow.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(deepCopyLaterPennationAngle,
                             deepCopyLater.characteristics().pennationAngle());
            EXPECT_NEAR(pennationAngle, 0.523599, requiredPrecision);
            EXPECT_NEAR(shallowCopyPennationAngle, 0.523599, requiredPrecision);
            EXPECT_NEAR(deepCopyNowPennationAngle, 0.15707963, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterPennationAngle, 0.15707963, requiredPrecision);
        }
    }

    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillType hillType(
            model.muscleGroup(muscleGroupForHillType).muscle(
                muscleForHillType));
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        biorbd::rigidbody::GeneralizedVelocity qDot(model);
        Q = Q.setOnes() / 10;
        qDot = qDot.setOnes() / 10;
        hillType.updateOrientations(model, Q);

        biorbd::muscles::HillType shallowCopy(hillType);
        biorbd::muscles::HillType deepCopyNow(hillType.DeepCopy());
        biorbd::muscles::HillType deepCopyLater;
        deepCopyLater.DeepCopy(hillType);

        {
            SCALAR_TO_DOUBLE(length, hillType.position().length());
            SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
            SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
            SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
            EXPECT_NEAR(length, 0.066381977535807504, requiredPrecision);
            EXPECT_NEAR(shallowCopyLength, 0.066381977535807504, requiredPrecision);
            EXPECT_NEAR(deepCopyNowLength, 0.066381977535807504, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterLength, 0.066381977535807504, requiredPrecision);
        }

        // Change the position of the insertion and pennation angle and compare again (length and insertion in Local)
        biorbd::muscles::Characteristics charac(hillType.characteristics());
        charac.setPennationAngle(0.523599);
        biorbd::utils::Vector3d insertion(hillType.position().insertionInLocal());
        insertion.set(0.5, 0.6, 0.7);
        biorbd::utils::String oldName(insertion.biorbd::utils::Node::name());
        biorbd::utils::String newName("MyNewName");
        insertion.setName(newName);
        hillType.updateOrientations(model, Q, qDot, 2);


        {
            SCALAR_TO_DOUBLE(length, hillType.position().length());
            SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
            SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
            SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
            EXPECT_NEAR(length, 0.07570761027741163, requiredPrecision);
            EXPECT_NEAR(shallowCopyLength, 0.07570761027741163, requiredPrecision);
            EXPECT_NEAR(deepCopyNowLength, 0.066381977535807504, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterLength, 0.066381977535807504, requiredPrecision);
            EXPECT_EQ(hillType.position().insertionInLocal().biorbd::utils::Node::name(),
                      newName);
            EXPECT_EQ(shallowCopy.position().insertionInLocal().biorbd::utils::Node::name(),
                      newName);
            EXPECT_EQ(deepCopyNow.position().insertionInLocal().biorbd::utils::Node::name(),
                      oldName);
            EXPECT_EQ(
                deepCopyLater.position().insertionInLocal().biorbd::utils::Node::name(),
                oldName);
        }
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillType hillType(
            model.muscleGroup(muscleGroupForHillType).muscle(
                muscleForHillType));

        biorbd::muscles::HillType shallowCopy(hillType);
        biorbd::muscles::HillType deepCopyNow(hillType.DeepCopy());
        biorbd::muscles::HillType deepCopyLater;
        deepCopyLater.DeepCopy(hillType);

        {
            SCALAR_TO_DOUBLE(excitation, hillType.state().excitation());
            SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyLaterExcitation, deepCopyLater.state().excitation());
            EXPECT_NEAR(excitation, 0., requiredPrecision);
            EXPECT_NEAR(shallowCopyExcitation, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
        }

        hillType.state().setExcitation(biorbd::utils::Scalar(5.));

        {
            SCALAR_TO_DOUBLE(excitation, hillType.state().excitation());
            SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyLaterExcitation, deepCopyLater.state().excitation());
            EXPECT_NEAR(excitation, 5., requiredPrecision);
            EXPECT_NEAR(shallowCopyExcitation, 5., requiredPrecision);
            EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
        }
    }
}

static unsigned int muscleGroupForHillThelenType(1);
static unsigned int muscleForHillThelenType(1);

TEST(hillThelenType, unitTest)
{
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));
        hillThelenType.setName("newName");
        EXPECT_STREQ(hillThelenType.name().c_str(), "newName");
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        biorbd::rigidbody::GeneralizedVelocity qDot(model);
        Q = Q.setOnes() / 10;
        qDot = qDot.setOnes() / 10;
        model.updateMuscles(Q, 2);
        hillThelenType.updateOrientations(model, Q, qDot);
        static double activationEmgForHillTypeTest(1.0);
        biorbd::muscles::StateDynamics emg(0, activationEmgForHillTypeTest);

        SCALAR_TO_DOUBLE(flce, hillThelenType.FlCE(emg));
        SCALAR_TO_DOUBLE(flpe, hillThelenType.FlPE());
        SCALAR_TO_DOUBLE(fvce, hillThelenType.FvCE());
        SCALAR_TO_DOUBLE(damping, hillThelenType.damping());
        SCALAR_TO_DOUBLE(force, hillThelenType.force(emg));
        EXPECT_NEAR(flce, 0.67988981401208015, requiredPrecision);
        EXPECT_NEAR(flpe, 0., requiredPrecision);
        EXPECT_NEAR(fvce, 1.000886825333013, requiredPrecision);
        EXPECT_NEAR(damping, 0.00019534599393617336, requiredPrecision);
        EXPECT_NEAR(force, 424.95358302550062, requiredPrecision);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));

        biorbd::muscles::HillThelenType hillThelenTypeNew(
            "newName",
            hillThelenType.position(),
            hillThelenType.characteristics());

        EXPECT_STREQ(hillThelenTypeNew.name().c_str(), "newName");
        EXPECT_EQ(hillThelenTypeNew.type(), biorbd::muscles::MUSCLE_TYPE::HILL_THELEN);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));

        biorbd::muscles::HillThelenType hillThelenTypeNew(
            "newName",
            hillThelenType.position(),
            hillThelenType.characteristics(),
            hillThelenType.state());

        EXPECT_STREQ(hillThelenTypeNew.name().c_str(), "newName");
        EXPECT_EQ(hillThelenTypeNew.type(), biorbd::muscles::MUSCLE_TYPE::HILL_THELEN);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));

        biorbd::muscles::HillThelenType hillThelenTypeNew(
            "newName",
            hillThelenType.position(),
            hillThelenType.characteristics(),
            hillThelenType.pathModifier());

        EXPECT_STREQ(hillThelenTypeNew.name().c_str(), "newName");
        EXPECT_EQ(hillThelenTypeNew.type(), biorbd::muscles::MUSCLE_TYPE::HILL_THELEN);
    }
}

TEST(hillThelenType, copy)
{
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        Q = Q.setOnes() / 10;

        biorbd::muscles::HillThelenType shallowCopy(hillThelenType);
        biorbd::muscles::HillThelenType deepCopyNow(hillThelenType.DeepCopy());
        biorbd::muscles::HillThelenType deepCopyLater;
        deepCopyLater.DeepCopy(hillThelenType);

        biorbd::utils::String originalName(hillThelenType.name());
        EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());

        biorbd::utils::String newName("MyNewMuscleName");
        hillThelenType.setName(newName);
        EXPECT_STREQ(hillThelenType.name().c_str(), newName.c_str());
        EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));

        biorbd::muscles::HillThelenType shallowCopy(hillThelenType);
        biorbd::muscles::HillThelenType deepCopyNow(hillThelenType.DeepCopy());
        biorbd::muscles::HillThelenType deepCopyLater;
        deepCopyLater.DeepCopy(hillThelenType);

        {
            SCALAR_TO_DOUBLE(pennationAngle,
                             hillThelenType.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(shallowCopyPennationAngle,
                             shallowCopy.characteristics().pennationAngle());
            EXPECT_NEAR(pennationAngle, 0.15707963, requiredPrecision);
            EXPECT_NEAR(shallowCopyPennationAngle, 0.15707963, requiredPrecision);
        }

        biorbd::muscles::Characteristics charac(hillThelenType.characteristics());
        charac.setPennationAngle(0.523599);

        {
            SCALAR_TO_DOUBLE(pennationAngle,
                             hillThelenType.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(shallowCopyPennationAngle,
                             shallowCopy.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(deepCopyNowPennationAngle,
                             deepCopyNow.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(deepCopyLaterPennationAngle,
                             deepCopyLater.characteristics().pennationAngle());
            EXPECT_NEAR(pennationAngle, 0.523599, requiredPrecision);
            EXPECT_NEAR(shallowCopyPennationAngle, 0.523599, requiredPrecision);
            EXPECT_NEAR(deepCopyNowPennationAngle, 0.15707963, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterPennationAngle, 0.15707963, requiredPrecision);
        }
    }

    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        biorbd::rigidbody::GeneralizedVelocity qDot(model);
        Q = Q.setOnes() / 10;
        qDot = qDot.setOnes() / 10;
        hillThelenType.updateOrientations(model, Q);

        biorbd::muscles::HillThelenType shallowCopy(hillThelenType);
        biorbd::muscles::HillThelenType deepCopyNow(hillThelenType.DeepCopy());
        biorbd::muscles::HillThelenType deepCopyLater;
        deepCopyLater.DeepCopy(hillThelenType);

        {
            SCALAR_TO_DOUBLE(length, hillThelenType.position().length());
            SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
            SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
            SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
            EXPECT_NEAR(length, 0.066381977535807504, requiredPrecision);
            EXPECT_NEAR(shallowCopyLength, 0.066381977535807504, requiredPrecision);
            EXPECT_NEAR(deepCopyNowLength, 0.066381977535807504, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterLength, 0.066381977535807504, requiredPrecision);
        }

        // Change the position of the insertion and pennation angle and compare again (length and insertion in Local)
        biorbd::muscles::Characteristics charac(hillThelenType.characteristics());
        charac.setPennationAngle(0.523599);
        biorbd::utils::Vector3d insertion(hillThelenType.position().insertionInLocal());
        insertion.set(0.5, 0.6, 0.7);
        biorbd::utils::String oldName(insertion.biorbd::utils::Node::name());
        biorbd::utils::String newName("MyNewName");
        insertion.setName(newName);
        hillThelenType.updateOrientations(model, Q, qDot, 2);

        {
            SCALAR_TO_DOUBLE(length, hillThelenType.position().length());
            SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
            SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
            SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
            EXPECT_NEAR(length, 0.07570761027741163, requiredPrecision);
            EXPECT_NEAR(shallowCopyLength, 0.07570761027741163, requiredPrecision);
            EXPECT_NEAR(deepCopyNowLength, 0.066381977535807504, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterLength, 0.066381977535807504, requiredPrecision);
            EXPECT_EQ(
                hillThelenType.position().insertionInLocal().biorbd::utils::Node::name(),
                newName);
            EXPECT_EQ(shallowCopy.position().insertionInLocal().biorbd::utils::Node::name(),
                      newName);
            EXPECT_EQ(deepCopyNow.position().insertionInLocal().biorbd::utils::Node::name(),
                      oldName);
            EXPECT_EQ(
                deepCopyLater.position().insertionInLocal().biorbd::utils::Node::name(),
                oldName);
        }
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));

        biorbd::muscles::HillThelenType shallowCopy(hillThelenType);
        biorbd::muscles::HillThelenType deepCopyNow(hillThelenType.DeepCopy());
        biorbd::muscles::HillThelenType deepCopyLater;
        deepCopyLater.DeepCopy(hillThelenType);

        {
            SCALAR_TO_DOUBLE(excitation, hillThelenType.state().excitation());
            SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyLaterExcitation, deepCopyLater.state().excitation());
            EXPECT_NEAR(excitation, 0., requiredPrecision);
            EXPECT_NEAR(shallowCopyExcitation, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
        }

        hillThelenType.state().setExcitation(biorbd::utils::Scalar(5.));

        {
            SCALAR_TO_DOUBLE(excitation, hillThelenType.state().excitation());
            SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyLaterExcitation, deepCopyLater.state().excitation());
            EXPECT_NEAR(excitation, 5., requiredPrecision);
            EXPECT_NEAR(shallowCopyExcitation, 5., requiredPrecision);
            EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
        }
    }
}

TEST(hillThelenTypeActive, unitTest)
{
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenActiveOnlyType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));
        hillThelenType.setName("newName");
        EXPECT_STREQ(hillThelenType.name().c_str(), "newName");
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenActiveOnlyType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        biorbd::rigidbody::GeneralizedVelocity qDot(model);
        Q = Q.setOnes() / 10;
        qDot = qDot.setOnes() / 10;
        model.updateMuscles(Q, 2);
        hillThelenType.updateOrientations(model, Q, qDot);
        static double activationEmgForHillTypeTest(1.0);
        biorbd::muscles::StateDynamics emg(0, activationEmgForHillTypeTest);

        SCALAR_TO_DOUBLE(flce, hillThelenType.FlCE(emg));
        SCALAR_TO_DOUBLE(flpe, hillThelenType.FlPE());
        SCALAR_TO_DOUBLE(fvce, hillThelenType.FvCE());
        SCALAR_TO_DOUBLE(damping, hillThelenType.damping());
        SCALAR_TO_DOUBLE(force, hillThelenType.force(emg));
        EXPECT_NEAR(flce, 0.67988981401208015, requiredPrecision);
        EXPECT_NEAR(flpe, 0., requiredPrecision);
        EXPECT_NEAR(fvce, 1.000886825333013, requiredPrecision);
        EXPECT_NEAR(damping, 0., requiredPrecision);
        EXPECT_NEAR(force, 424.83162852148627, requiredPrecision);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenActiveOnlyType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));

        biorbd::muscles::HillThelenActiveOnlyType hillThelenTypeNew(
            "newName",
            hillThelenType.position(),
            hillThelenType.characteristics());

        EXPECT_STREQ(hillThelenTypeNew.name().c_str(), "newName");
        EXPECT_EQ(hillThelenTypeNew.type(),
                  biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenActiveOnlyType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));

        biorbd::muscles::HillThelenActiveOnlyType hillThelenTypeNew(
            "newName",
            hillThelenType.position(),
            hillThelenType.characteristics(),
            hillThelenType.state());

        EXPECT_STREQ(hillThelenTypeNew.name().c_str(), "newName");
        EXPECT_EQ(hillThelenTypeNew.type(),
                  biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenActiveOnlyType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));

        biorbd::muscles::HillThelenActiveOnlyType hillThelenTypeNew(
            "newName",
            hillThelenType.position(),
            hillThelenType.characteristics(),
            hillThelenType.pathModifier());

        EXPECT_STREQ(hillThelenTypeNew.name().c_str(), "newName");
        EXPECT_EQ(hillThelenTypeNew.type(),
                  biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE);
    }
}

TEST(hillThelenActiveType, copy)
{
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenActiveOnlyType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        Q = Q.setOnes() / 10;

        biorbd::muscles::HillThelenActiveOnlyType shallowCopy(hillThelenType);
        biorbd::muscles::HillThelenActiveOnlyType deepCopyNow(
            hillThelenType.DeepCopy());
        biorbd::muscles::HillThelenActiveOnlyType deepCopyLater;
        deepCopyLater.DeepCopy(hillThelenType);

        biorbd::utils::String originalName(hillThelenType.name());
        EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());

        biorbd::utils::String newName("MyNewMuscleName");
        hillThelenType.setName(newName);
        EXPECT_STREQ(hillThelenType.name().c_str(), newName.c_str());
        EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenActiveOnlyType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));

        biorbd::muscles::HillThelenActiveOnlyType shallowCopy(hillThelenType);
        biorbd::muscles::HillThelenActiveOnlyType deepCopyNow(
            hillThelenType.DeepCopy());
        biorbd::muscles::HillThelenActiveOnlyType deepCopyLater;
        deepCopyLater.DeepCopy(hillThelenType);

        {
            SCALAR_TO_DOUBLE(pennationAngle,
                             hillThelenType.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(shallowCopyPennationAngle,
                             shallowCopy.characteristics().pennationAngle());
            EXPECT_NEAR(pennationAngle, 0.15707963, requiredPrecision);
            EXPECT_NEAR(shallowCopyPennationAngle, 0.15707963, requiredPrecision);
        }

        biorbd::muscles::Characteristics charac(hillThelenType.characteristics());
        charac.setPennationAngle(0.523599);

        {
            SCALAR_TO_DOUBLE(pennationAngle,
                             hillThelenType.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(shallowCopyPennationAngle,
                             shallowCopy.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(deepCopyNowPennationAngle,
                             deepCopyNow.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(deepCopyLaterPennationAngle,
                             deepCopyLater.characteristics().pennationAngle());
            EXPECT_NEAR(pennationAngle, 0.523599, requiredPrecision);
            EXPECT_NEAR(shallowCopyPennationAngle, 0.523599, requiredPrecision);
            EXPECT_NEAR(deepCopyNowPennationAngle, 0.15707963, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterPennationAngle, 0.15707963, requiredPrecision);
        }
    }

    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenActiveOnlyType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));
        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        biorbd::rigidbody::GeneralizedVelocity qDot(model);
        Q = Q.setOnes() / 10;
        qDot = qDot.setOnes() / 10;
        hillThelenType.updateOrientations(model, Q);

        biorbd::muscles::HillThelenActiveOnlyType shallowCopy(hillThelenType);
        biorbd::muscles::HillThelenActiveOnlyType deepCopyNow(
            hillThelenType.DeepCopy());
        biorbd::muscles::HillThelenActiveOnlyType deepCopyLater;
        deepCopyLater.DeepCopy(hillThelenType);

        {
            SCALAR_TO_DOUBLE(length, hillThelenType.position().length());
            SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
            SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
            SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
            EXPECT_NEAR(length, 0.066381977535807504, requiredPrecision);
            EXPECT_NEAR(shallowCopyLength, 0.066381977535807504, requiredPrecision);
            EXPECT_NEAR(deepCopyNowLength, 0.066381977535807504, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterLength, 0.066381977535807504, requiredPrecision);
        }

        // Change the position of the insertion and pennation angle and compare again (length and insertion in Local)
        biorbd::muscles::Characteristics charac(hillThelenType.characteristics());
        charac.setPennationAngle(0.523599);
        biorbd::utils::Vector3d insertion(hillThelenType.position().insertionInLocal());
        insertion.set(0.5, 0.6, 0.7);
        biorbd::utils::String oldName(insertion.biorbd::utils::Node::name());
        biorbd::utils::String newName("MyNewName");
        insertion.setName(newName);
        hillThelenType.updateOrientations(model, Q, qDot, 2);

        {
            SCALAR_TO_DOUBLE(length, hillThelenType.position().length());
            SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.position().length());
            SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.position().length());
            SCALAR_TO_DOUBLE(deepCopyLaterLength, deepCopyLater.position().length());
            EXPECT_NEAR(length, 0.07570761027741163, requiredPrecision);
            EXPECT_NEAR(shallowCopyLength, 0.07570761027741163, requiredPrecision);
            EXPECT_NEAR(deepCopyNowLength, 0.066381977535807504, requiredPrecision);
            EXPECT_NEAR(deepCopyLaterLength, 0.066381977535807504, requiredPrecision);
            EXPECT_EQ(
                hillThelenType.position().insertionInLocal().biorbd::utils::Node::name(),
                newName);
            EXPECT_EQ(shallowCopy.position().insertionInLocal().biorbd::utils::Node::name(),
                      newName);
            EXPECT_EQ(deepCopyNow.position().insertionInLocal().biorbd::utils::Node::name(),
                      oldName);
            EXPECT_EQ(
                deepCopyLater.position().insertionInLocal().biorbd::utils::Node::name(),
                oldName);
        }
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenActiveOnlyType hillThelenType(
            model.muscleGroup(muscleGroupForHillThelenType).muscle(
                muscleForHillThelenType));

        biorbd::muscles::HillThelenActiveOnlyType shallowCopy(hillThelenType);
        biorbd::muscles::HillThelenActiveOnlyType deepCopyNow(
            hillThelenType.DeepCopy());
        biorbd::muscles::HillThelenActiveOnlyType deepCopyLater;
        deepCopyLater.DeepCopy(hillThelenType);

        {
            SCALAR_TO_DOUBLE(excitation, hillThelenType.state().excitation());
            SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyLaterExcitation, deepCopyLater.state().excitation());
            EXPECT_NEAR(excitation, 0., requiredPrecision);
            EXPECT_NEAR(shallowCopyExcitation, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
        }

        hillThelenType.state().setExcitation(biorbd::utils::Scalar(5.));

        {
            SCALAR_TO_DOUBLE(excitation, hillThelenType.state().excitation());
            SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyLaterExcitation, deepCopyLater.state().excitation());
            EXPECT_NEAR(excitation, 5., requiredPrecision);
            EXPECT_NEAR(shallowCopyExcitation, 5., requiredPrecision);
            EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
        }
    }
}

TEST(DynamicState, Normal)
{
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::StateDynamics state(0.8, 0.5);

        const biorbd::muscles::Muscle& m(model.muscle(0));
        SCALAR_TO_DOUBLE(actDot, m.activationDot(state));
        EXPECT_NEAR(actDot, 24.0, requiredPrecision);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::StateDynamics state(0.3, 0.5);

        const biorbd::muscles::Muscle& m(model.muscle(0));
        SCALAR_TO_DOUBLE(actDot, m.activationDot(state));
        EXPECT_NEAR(actDot, -6.25, requiredPrecision);
    }
}

TEST(DynamicState, Buchanan)
{
    {
        biorbd::Model model(modelPathForBuchananDynamics);
        static_cast<biorbd::muscles::StateDynamicsBuchanan&>(
            model.muscles()[2]->state()).shapeFactor(5.5);

        // Test shape factor changes
        biorbd::utils::Scalar shapeFactor0 =
            static_cast<biorbd::muscles::StateDynamicsBuchanan&>(
                model.muscles()[0]->state()).shapeFactor();
        SCALAR_TO_DOUBLE(shapeFactor0_double, shapeFactor0);
        EXPECT_NEAR(shapeFactor0_double, -3, requiredPrecision);

        biorbd::utils::Scalar shapeFactor1 =
            static_cast<biorbd::muscles::StateDynamicsBuchanan&>(
                model.muscles()[1]->state()).shapeFactor();
        SCALAR_TO_DOUBLE(shapeFactor1_double, shapeFactor1);
        EXPECT_NEAR(shapeFactor1_double, 10, requiredPrecision);

        biorbd::utils::Scalar shapeFactor2 =
            static_cast<biorbd::muscles::StateDynamicsBuchanan&>(
                model.muscles()[2]->state()).shapeFactor();
        SCALAR_TO_DOUBLE(shapeFactor2_double, shapeFactor2);
        EXPECT_NEAR(shapeFactor2_double, 5.5, requiredPrecision);

        biorbd::muscles::StateDynamics state(0.8, 0.5);

        const biorbd::muscles::Muscle& m(model.muscle(0));
        SCALAR_TO_DOUBLE(actDot, m.activationDot(state));
        EXPECT_NEAR(actDot, -7.592740648890816, requiredPrecision);
    }
    {
        biorbd::Model model(modelPathForBuchananDynamics);
        biorbd::muscles::StateDynamics state(0.3, 0.5);

        const biorbd::muscles::Muscle& m(model.muscle(0));
        SCALAR_TO_DOUBLE(actDot, m.activationDot(state));
        EXPECT_NEAR(actDot, -11.656766195499843, requiredPrecision);
    }
}

TEST(DynamicState, DeGroote)
{
    {
        biorbd::Model model(modelPathForDeGrooteDynamics);
        biorbd::muscles::StateDynamics state(0.8, 0.5);

        const biorbd::muscles::Muscle& m(model.muscle(0));
        SCALAR_TO_DOUBLE(actDot, m.activationDot(state));
        EXPECT_NEAR(actDot, 16.906809211183873, requiredPrecision);
    }
    {
        biorbd::Model model(modelPathForDeGrooteDynamics);
        biorbd::muscles::StateDynamics state(0.3, 0.5);

        const biorbd::muscles::Muscle& m(model.muscle(0));
        SCALAR_TO_DOUBLE(actDot, m.activationDot(state));
        EXPECT_NEAR(actDot, -11.027512997920336, requiredPrecision);
    }
}

static unsigned int muscleGroupForHillThelenTypeFatigable(1);
static unsigned int muscleForHillThelenTypeFatigable(1);

TEST(hillThelenTypeFatigable, unitTest)
{
    {
        biorbd::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;
        hillThelenTypeFatigable.setName("newName");
        EXPECT_STREQ(hillThelenTypeFatigable.name().c_str(), "newName");
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        EXPECT_THROW(biorbd::muscles::HillThelenTypeFatigable hillThelenTypeFatigable(
                         model.muscleGroup(muscleGroupForHillThelenTypeFatigable).muscle(
                             muscleForHillThelenTypeFatigable)), std::bad_cast);
    }
    {
        biorbd::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

        biorbd::muscles::HillThelenTypeFatigable hillThelenTypeFatigableNew(
            "newName",
            hillThelenTypeFatigable.position(),
            hillThelenTypeFatigable.characteristics(),
            biorbd::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

        EXPECT_STREQ(hillThelenTypeFatigableNew.name().c_str(), "newName");
        EXPECT_EQ(hillThelenTypeFatigableNew.type(),
                  biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE);
    }

    {
        biorbd::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

        biorbd::muscles::HillThelenTypeFatigable hillThelenTypeFatigableNew(
            "newName",
            hillThelenTypeFatigable.position(),
            hillThelenTypeFatigable.characteristics(),
            hillThelenTypeFatigable.state());

        EXPECT_STREQ(hillThelenTypeFatigableNew.name().c_str(), "newName");
        EXPECT_EQ(hillThelenTypeFatigableNew.type(),
                  biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE);
    }
    {
        biorbd::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

        biorbd::muscles::HillThelenTypeFatigable hillThelenTypeFatigableNew(
            "newName",
            hillThelenTypeFatigable.position(),
            hillThelenTypeFatigable.characteristics(),
            hillThelenTypeFatigable.pathModifier(),
            biorbd::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

        EXPECT_STREQ(hillThelenTypeFatigableNew.name().c_str(), "newName");
        EXPECT_EQ(hillThelenTypeFatigableNew.type(),
                  biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE);
    }
    {
        biorbd::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

        biorbd::muscles::HillThelenTypeFatigable hillThelenTypeFatigableNew(
            "nameMuscle",
            hillThelenTypeFatigable.position(),
            hillThelenTypeFatigable.characteristics(),
            biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

        EXPECT_EQ(hillThelenTypeFatigableNew.type(),
                  biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE);
    }
    {
        biorbd::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

        biorbd::muscles::HillThelenTypeFatigable hillThelenTypeFatigableNew(
            "nameMuscle",
            hillThelenTypeFatigable.position(),
            hillThelenTypeFatigable.characteristics(),
            hillThelenTypeFatigable.pathModifier(),
            biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

        EXPECT_EQ(hillThelenTypeFatigableNew.type(),
                  biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE);
    }
}

TEST(hillThelenTypeFatigable, copy)
{
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

        biorbd::rigidbody::GeneralizedCoordinates Q(model);
        Q = Q.setOnes() / 10;

        biorbd::muscles::HillThelenTypeFatigable shallowCopy(hillThelenTypeFatigable);
        biorbd::muscles::HillThelenTypeFatigable deepCopyNow(
            hillThelenTypeFatigable.DeepCopy());
        biorbd::muscles::HillThelenTypeFatigable deepCopyLater;
        deepCopyLater.DeepCopy(hillThelenTypeFatigable);

        biorbd::utils::String originalName(hillThelenTypeFatigable.name());
        EXPECT_STREQ(shallowCopy.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());

        biorbd::utils::String newName("MyNewMuscleName");
        hillThelenTypeFatigable.setName(newName);
        EXPECT_STREQ(hillThelenTypeFatigable.name().c_str(), newName.c_str());
        EXPECT_STREQ(shallowCopy.name().c_str(), newName.c_str());
        EXPECT_STREQ(deepCopyNow.name().c_str(), originalName.c_str());
        EXPECT_STREQ(deepCopyLater.name().c_str(), originalName.c_str());
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

        biorbd::muscles::HillThelenTypeFatigable shallowCopy(hillThelenTypeFatigable);
        biorbd::muscles::HillThelenTypeFatigable deepCopyNow(
            hillThelenTypeFatigable.DeepCopy());
        biorbd::muscles::HillThelenTypeFatigable deepCopyLater;
        deepCopyLater.DeepCopy(hillThelenTypeFatigable);

        {
            SCALAR_TO_DOUBLE(pennationAngle,
                             hillThelenTypeFatigable.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(shallowCopyPennationAngle,
                             shallowCopy.characteristics().pennationAngle());
            EXPECT_NEAR(pennationAngle, 0., requiredPrecision);
            EXPECT_NEAR(shallowCopyPennationAngle, 0., requiredPrecision);
        }
        biorbd::muscles::Characteristics charac(
            hillThelenTypeFatigable.characteristics());
        charac.setPennationAngle(0.523599);
        {
            SCALAR_TO_DOUBLE(pennationAngle,
                             hillThelenTypeFatigable.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(shallowCopyPennationAngle,
                             shallowCopy.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(deepCopyNowPennationAngle,
                             deepCopyNow.characteristics().pennationAngle());
            SCALAR_TO_DOUBLE(deepCopyLaterPennationAngle,
                             deepCopyLater.characteristics().pennationAngle());
            EXPECT_NEAR(pennationAngle, 0.523599, requiredPrecision);
            EXPECT_NEAR(shallowCopyPennationAngle, 0.523599, requiredPrecision);
            EXPECT_NEAR(deepCopyNowPennationAngle, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyLaterPennationAngle, 0., requiredPrecision);
        }
    }

    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::HillThelenTypeFatigable hillThelenTypeFatigable;

        biorbd::muscles::HillThelenTypeFatigable shallowCopy(hillThelenTypeFatigable);
        biorbd::muscles::HillThelenTypeFatigable deepCopyNow(
            hillThelenTypeFatigable.DeepCopy());
        biorbd::muscles::HillThelenTypeFatigable deepCopyLater;
        deepCopyLater.DeepCopy(hillThelenTypeFatigable);

        {
            SCALAR_TO_DOUBLE(excitation, hillThelenTypeFatigable.state().excitation());
            SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyLaterExcitation, deepCopyLater.state().excitation());
            EXPECT_NEAR(excitation, 0., requiredPrecision);
            EXPECT_NEAR(shallowCopyExcitation, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
        }

        hillThelenTypeFatigable.state().setExcitation(biorbd::utils::Scalar(5.));

        {
            SCALAR_TO_DOUBLE(excitation, hillThelenTypeFatigable.state().excitation());
            SCALAR_TO_DOUBLE(shallowCopyExcitation, shallowCopy.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyNowExcitation, deepCopyNow.state().excitation());
            SCALAR_TO_DOUBLE(deepCopyLaterExcitation, deepCopyLater.state().excitation());
            EXPECT_NEAR(excitation, 5., requiredPrecision);
            EXPECT_NEAR(shallowCopyExcitation, 5., requiredPrecision);
            EXPECT_NEAR(deepCopyNowExcitation, 0., requiredPrecision);
            EXPECT_NEAR(deepCopyLaterExcitation, 0., requiredPrecision);
        }
    }
}

#ifndef BIORBD_USE_CASADI_MATH
TEST(FatigueState, unitTest)
{
    biorbd::muscles::FatigueState fatigueState;
    fatigueState.setState(0.0, 1.0, 0.0);

    SCALAR_TO_DOUBLE(activeFibers, fatigueState.activeFibers());
    SCALAR_TO_DOUBLE(fatiguedFibers, fatigueState.fatiguedFibers());
    SCALAR_TO_DOUBLE(restingFibers, fatigueState.restingFibers());
    EXPECT_EQ(activeFibers, 0.0);
    EXPECT_EQ(fatiguedFibers, 1.);
    EXPECT_EQ(restingFibers, 0.);
}
#endif

TEST(FatigueState, copy)
{
    biorbd::muscles::FatigueState fatigueState;

    biorbd::muscles::FatigueState shallowCopy(fatigueState);
    biorbd::muscles::FatigueState deepCopyNow(fatigueState.DeepCopy());
    biorbd::muscles::FatigueState deepCopyLater;
    deepCopyLater.DeepCopy(fatigueState);


    {
        SCALAR_TO_DOUBLE(fatiguedFibers, fatigueState.fatiguedFibers());
        SCALAR_TO_DOUBLE(shallowCopyFatigued, shallowCopy.fatiguedFibers());
        SCALAR_TO_DOUBLE(deepCopyNowFatigued, deepCopyNow.fatiguedFibers());
        SCALAR_TO_DOUBLE(deepCopyLaterFatigued, deepCopyLater.fatiguedFibers());
        EXPECT_EQ(fatiguedFibers, 0.0);
        EXPECT_EQ(shallowCopyFatigued, 0.0);
        EXPECT_EQ(deepCopyNowFatigued, 0.0);
        EXPECT_EQ(deepCopyLaterFatigued, 0.0);
    }

#ifndef BIORBD_USE_CASADI_MATH
    fatigueState.setState(0.0, 1.0, 0.0);
    //change state
    {
        SCALAR_TO_DOUBLE(fatiguedFibers, fatigueState.fatiguedFibers());
        SCALAR_TO_DOUBLE(shallowCopyFatigued, shallowCopy.fatiguedFibers());
        SCALAR_TO_DOUBLE(deepCopyNowFatigued, deepCopyNow.fatiguedFibers());
        SCALAR_TO_DOUBLE(deepCopyLaterFatigued, deepCopyLater.fatiguedFibers());
        EXPECT_EQ(fatiguedFibers, 1.0);
        EXPECT_EQ(shallowCopyFatigued, 1.0);
        EXPECT_EQ(deepCopyNowFatigued, 0.0);
        EXPECT_EQ(deepCopyLaterFatigued, 0.0);
    }
#endif
}

static std::string modelPathForXiaDerivativeTest("models/arm26.bioMod");
static unsigned int muscleGroupForXiaDerivativeTest(0);
static unsigned int muscleForXiaDerivativeTest(0);
#ifndef BIORBD_USE_CASADI_MATH
    static double activationEmgForXiaDerivativeTest(1.0);
    static double currentActiveFibersForXiaDerivativeTest(0.9);
    static double currentFatiguedFibersForXiaDerivativeTest(0.0);
    static double currentRestingFibersForXiaDerivativeTest(0.1);
    static double expectedActivationDotForXiaDerivativeTest(0.991);
    static double expectedFatiguedDotForXiaDerivativeTest(0.009);
    static double expectedRestingDotForXiaDerivativeTest(-1.0);
    static double positiveFibersQuantityForFatigueXiaSetStateLimitsTest(1.0);
    static double negativeFibersQuantityForFatigueXiaSetStateLimitsTest(-1.0);
    static double excessiveFibersQuantityForFatigueXiaSetStateLimitsTest(1.5);
#endif

TEST(FatigueDynamiqueState, DeepCopy)
{
    // Prepare the model
    biorbd::Model model(modelPathForXiaDerivativeTest);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity QDot(model);
    Q.setZero();
    QDot.setZero();
    model.updateMuscles(Q, QDot, true);

    {
        biorbd::muscles::HillThelenTypeFatigable muscle(model.muscleGroup(
                    muscleGroupForXiaDerivativeTest).muscle(
                    muscleForXiaDerivativeTest));
        biorbd::muscles::FatigueDynamicStateXia fatigueDynamicStateXia;
        // Sanity check for the fatigue model
        EXPECT_EQ(fatigueDynamicStateXia.getType(),
                  biorbd::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

        // Initial value sanity check
        {
            SCALAR_TO_DOUBLE(val, fatigueDynamicStateXia.activeFibersDot());
            EXPECT_NEAR(val, 0, requiredPrecision);
        }

#ifndef BIORBD_USE_CASADI_MATH
        // Apply the derivative
        biorbd::muscles::StateDynamics emg(0,
                                           activationEmgForXiaDerivativeTest); // Set target
        fatigueDynamicStateXia.setState(currentActiveFibersForXiaDerivativeTest,
                                        currentFatiguedFibersForXiaDerivativeTest,
                                        currentRestingFibersForXiaDerivativeTest);
        fatigueDynamicStateXia.timeDerivativeState(emg,
                model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(
                    muscleForXiaDerivativeTest).characteristics());

        // Check the values
        EXPECT_NEAR(fatigueDynamicStateXia.activeFibersDot(),
                    expectedActivationDotForXiaDerivativeTest, requiredPrecision);

        // Make copies
        biorbd::muscles::FatigueDynamicStateXia shallowCopy(fatigueDynamicStateXia);
        biorbd::muscles::FatigueDynamicStateXia deepCopyNow(
            fatigueDynamicStateXia.DeepCopy());
        biorbd::muscles::FatigueDynamicStateXia deepCopyLater;
        deepCopyLater.DeepCopy(fatigueDynamicStateXia);

        // Check the values
        EXPECT_NEAR(shallowCopy.activeFibersDot(),
                    expectedActivationDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(deepCopyNow.activeFibersDot(),
                    expectedActivationDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(deepCopyLater.activeFibersDot(),
                    expectedActivationDotForXiaDerivativeTest, requiredPrecision);

        // Change state to change Fibers Dots
        fatigueDynamicStateXia.setState(0.5, 0.5, 0.0);
        fatigueDynamicStateXia.timeDerivativeState(emg,
                model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(
                    muscleForXiaDerivativeTest).characteristics());

        // Check the new values
        EXPECT_NEAR(fatigueDynamicStateXia.activeFibersDot(), -0.0050000000000000001,
                    requiredPrecision);
        EXPECT_NEAR(shallowCopy.activeFibersDot(), -0.0050000000000000001,
                    requiredPrecision);
        EXPECT_NEAR(deepCopyNow.activeFibersDot(),
                    expectedActivationDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(deepCopyLater.activeFibersDot(),
                    expectedActivationDotForXiaDerivativeTest, requiredPrecision);
#endif
    }
}

TEST(FatigueParameters, unitTest)
{
    biorbd::muscles::FatigueParameters fatigueParameters;
    fatigueParameters.setFatigueRate(25.0);
    SCALAR_TO_DOUBLE(fatigueRate, fatigueParameters.fatigueRate());
    EXPECT_EQ(fatigueRate, 25.0);

    fatigueParameters.setRecoveryRate(10.0);
    SCALAR_TO_DOUBLE(recoveryRate, fatigueParameters.recoveryRate());
    EXPECT_EQ(recoveryRate, 10.0);

    fatigueParameters.setDevelopFactor(3.0);
    SCALAR_TO_DOUBLE(developFactor, fatigueParameters.developFactor());
    EXPECT_EQ(developFactor, 3.0);

    fatigueParameters.setRecoveryFactor(2.0);
    SCALAR_TO_DOUBLE(recoveryFactor, fatigueParameters.recoveryFactor());
    EXPECT_EQ(recoveryFactor, 2.0);
}

TEST(FatigueParemeters, copy)
{
    biorbd::muscles::FatigueParameters fatigueParameters(1.0, 2.0, 3.0, 4.0);
    biorbd::muscles::FatigueParameters shallowCopy(fatigueParameters);
    biorbd::muscles::FatigueParameters deepCopyNow(fatigueParameters.DeepCopy());
    biorbd::muscles::FatigueParameters deepCopyLater;
    deepCopyLater.DeepCopy(fatigueParameters);

    {
        SCALAR_TO_DOUBLE(recoveryRate, fatigueParameters.recoveryRate());
        SCALAR_TO_DOUBLE(shallowCopyRecoveryRate, shallowCopy.recoveryRate());
        SCALAR_TO_DOUBLE(deepCopyNowRecoveryRate, deepCopyNow.recoveryRate());
        SCALAR_TO_DOUBLE(deepCopyLaterRecoveryRate, deepCopyLater.recoveryRate());
        EXPECT_NEAR(recoveryRate, 2., requiredPrecision);
        EXPECT_NEAR(shallowCopyRecoveryRate, 2., requiredPrecision);
        EXPECT_NEAR(deepCopyNowRecoveryRate, 2., requiredPrecision);
        EXPECT_NEAR(deepCopyLaterRecoveryRate, 2., requiredPrecision);
    }

    fatigueParameters.setRecoveryRate(5.0);

    {
        SCALAR_TO_DOUBLE(recoveryRate, fatigueParameters.recoveryRate());
        SCALAR_TO_DOUBLE(shallowCopyRecoveryRate, shallowCopy.recoveryRate());
        SCALAR_TO_DOUBLE(deepCopyNowRecoveryRate, deepCopyNow.recoveryRate());
        SCALAR_TO_DOUBLE(deepCopyLaterRecoveryRate, deepCopyLater.recoveryRate());
        EXPECT_NEAR(recoveryRate, 5., requiredPrecision);
        EXPECT_NEAR(shallowCopyRecoveryRate, 5., requiredPrecision);
        EXPECT_NEAR(deepCopyNowRecoveryRate, 2., requiredPrecision);
        EXPECT_NEAR(deepCopyLaterRecoveryRate, 2., requiredPrecision);
    }
}

TEST(MuscleGroup, unitTest)
{
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

        muscleGroup.setName("newName");
        EXPECT_STREQ(muscleGroup.name().c_str(), "newName");
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

        muscleGroup.setOrigin("newOriginName");
        EXPECT_STREQ(muscleGroup.origin().c_str(), "newOriginName");
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

        muscleGroup.setInsertion("newInsertionName");
        EXPECT_STREQ(muscleGroup.insertion().c_str(), "newInsertionName");
    }

    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

        muscleGroup.addMuscle("newMuscleName",
                              biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR,
                              model.muscleGroup(0).muscle(0).position(),
                              model.muscleGroup(0).muscle(0).characteristics(),
                              biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

        // Check the id of the last muscle added
        EXPECT_NEAR(muscleGroup.nbMuscles(), 4, requiredPrecision);
        int idNewMuscle(muscleGroup.muscleID("newMuscleName"));
        EXPECT_NEAR(idNewMuscle, 3, requiredPrecision);

        // Fetch new muscle from muscle
        EXPECT_STREQ(muscleGroup.muscle(3).name().c_str(), "newMuscleName");
    }
}

TEST(MuscleGroup, AddMuscle)
{
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

        biorbd::muscles::IdealizedActuator muscleToAdd;
        muscleGroup.addMuscle(muscleToAdd);

        //Check number of muscle
        EXPECT_NEAR(muscleGroup.nbMuscles(), 4, requiredPrecision);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

        //Check number of muscle
        EXPECT_NEAR(muscleGroup.nbMuscles(), 3, requiredPrecision);

        // Add muscle to muscle group
        muscleGroup.addMuscle("newMuscleName",
                              biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR,
                              model.muscleGroup(0).muscle(0).position(),
                              model.muscleGroup(0).muscle(0).characteristics(),
                              biorbd::muscles::STATE_TYPE::SIMPLE_STATE,
                              biorbd::muscles::STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);

        // Check the number of muscles again
        EXPECT_NEAR(muscleGroup.nbMuscles(), 4, requiredPrecision);

        // Add HILL muscle to muscle group
        muscleGroup.addMuscle("newHillMuscle",
                              biorbd::muscles::MUSCLE_TYPE::HILL,
                              model.muscleGroup(0).muscle(0).position(),
                              model.muscleGroup(0).muscle(0).characteristics(),
                              biorbd::muscles::STATE_TYPE::DYNAMIC,
                              biorbd::muscles::STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);

        // Check the number of muscles again
        EXPECT_NEAR(muscleGroup.nbMuscles(), 5, requiredPrecision);

        // Add HILL THELEN muscle to muscle group
        muscleGroup.addMuscle("newHillThelenMuscle",
                              biorbd::muscles::MUSCLE_TYPE::HILL_THELEN,
                              model.muscleGroup(0).muscle(0).position(),
                              model.muscleGroup(0).muscle(0).characteristics(),
                              biorbd::muscles::STATE_TYPE::BUCHANAN,
                              biorbd::muscles::STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);

        // Check the number of muscles again
        EXPECT_NEAR(muscleGroup.nbMuscles(), 6, requiredPrecision);

        // Add muscle to muscle group
        muscleGroup.addMuscle("newHillThelenFatigable",
                              biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE,
                              model.muscleGroup(0).muscle(0).position(),
                              model.muscleGroup(0).muscle(0).characteristics(),
                              biorbd::muscles::STATE_TYPE::SIMPLE_STATE,
                              biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

        // Check the number of muscles again
        EXPECT_NEAR(muscleGroup.nbMuscles(), 7, requiredPrecision);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

        //Check number of muscle
        EXPECT_NEAR(muscleGroup.nbMuscles(), 3, requiredPrecision);

        // Add muscle to muscle group
        muscleGroup.addMuscle("newMuscleName",
                              biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR,
                              model.muscleGroup(0).muscle(0).position(),
                              model.muscleGroup(0).muscle(0).characteristics(),
                              model.muscleGroup(0).muscle(0).pathModifier(),
                              biorbd::muscles::STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);

        // Check the number of muscles again
        EXPECT_NEAR(muscleGroup.nbMuscles(), 4, requiredPrecision);
    }
}

TEST(MuscleGroup, DeepCopy)
{
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

        biorbd::muscles::MuscleGroup shallowCopy(muscleGroup);
        biorbd::muscles::MuscleGroup deepCopyNow(muscleGroup.DeepCopy());
        biorbd::muscles::MuscleGroup deepCopyLater;
        deepCopyLater.DeepCopy(muscleGroup);

        EXPECT_STREQ(muscleGroup.name().c_str(), "base_to_r_ulna_radius_hand");
        EXPECT_STREQ(shallowCopy.name().c_str(), "base_to_r_ulna_radius_hand");
        EXPECT_STREQ(deepCopyNow.name().c_str(), "base_to_r_ulna_radius_hand");
        EXPECT_STREQ(deepCopyLater.name().c_str(), "base_to_r_ulna_radius_hand");

        // Set new name
        muscleGroup.setName("newMuscleGroupName");

        EXPECT_STREQ(muscleGroup.name().c_str(), "newMuscleGroupName");
        EXPECT_STREQ(shallowCopy.name().c_str(), "newMuscleGroupName");
        EXPECT_STREQ(deepCopyNow.name().c_str(), "base_to_r_ulna_radius_hand");
        EXPECT_STREQ(deepCopyLater.name().c_str(), "base_to_r_ulna_radius_hand");
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));

        muscleGroup.addMuscle("newIdealizedActuator",
                              biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR,
                              model.muscleGroup(0).muscle(0).position(),
                              model.muscleGroup(0).muscle(0).characteristics(),
                              biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

        muscleGroup.addMuscle("newHillActuator",
                              biorbd::muscles::MUSCLE_TYPE::HILL,
                              model.muscleGroup(0).muscle(0).position(),
                              model.muscleGroup(0).muscle(0).characteristics(),
                              biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

        biorbd::muscles::MuscleGroup shallowCopy(muscleGroup);
        biorbd::muscles::MuscleGroup deepCopyNow(muscleGroup.DeepCopy());
        biorbd::muscles::MuscleGroup deepCopyLater;
        deepCopyLater.DeepCopy(muscleGroup);

        EXPECT_STREQ(muscleGroup.muscle(4).name().c_str(), "newHillActuator");
        EXPECT_STREQ(shallowCopy.muscle(4).name().c_str(), "newHillActuator");
        EXPECT_STREQ(deepCopyNow.muscle(4).name().c_str(), "newHillActuator");
        EXPECT_STREQ(deepCopyLater.muscle(4).name().c_str(), "newHillActuator");
    }
}

TEST(MuscleGroup, errors)
{
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));
        EXPECT_THROW(muscleGroup.addMuscle("noTypeMuscle",
                                           biorbd::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE,
                                           model.muscleGroup(0).muscle(0).position(),
                                           model.muscleGroup(0).muscle(0).characteristics(),
                                           biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE), std::runtime_error);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));
        EXPECT_THROW(muscleGroup.muscle(3), std::runtime_error);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));
        EXPECT_THROW(muscleGroup.addMuscle("noTypeMuscle",
                                           biorbd::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE,
                                           model.muscleGroup(0).muscle(0).position(),
                                           model.muscleGroup(0).muscle(0).characteristics(),
                                           biorbd::muscles::STATE_TYPE::SIMPLE_STATE,
                                           biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE), std::runtime_error);
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::MuscleGroup muscleGroup(model.muscleGroup(0));
        EXPECT_THROW(muscleGroup.addMuscle("noTypeMuscle",
                                           biorbd::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE,
                                           model.muscleGroup(0).muscle(0).position(),
                                           model.muscleGroup(0).muscle(0).characteristics(),
                                           model.muscleGroup(0).muscle(0).pathModifier(),
                                           biorbd::muscles::STATE_TYPE::SIMPLE_STATE,
                                           biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE), std::runtime_error);
    }
}

TEST(Muscles, unitTest)
{
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::Muscles muscles;
        muscles.addMuscleGroup("muscleGroupName", "originName", "insertionName");

        EXPECT_STREQ(muscles.muscleGroup(0).name().c_str(), "muscleGroupName");
        EXPECT_STREQ(muscles.muscleGroups()[0].name().c_str(), "muscleGroupName");
        EXPECT_STREQ(muscles.muscleGroup("muscleGroupName").origin().c_str(),
                     "originName");
    }
    {
        biorbd::Model model(modelPathForMuscleForce);
        biorbd::muscles::Muscles muscles;
        muscles.addMuscleGroup("muscleGroupName", "originName", "insertionName");
        muscles.muscleGroup(0).addMuscle("newIdealizedActuator",
                                         biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR,
                                         model.muscleGroup(0).muscle(0).position(),
                                         model.muscleGroup(0).muscle(0).characteristics(),
                                         biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

        EXPECT_NEAR(muscles.muscleNames().size(), 1., requiredPrecision);
    }
}

TEST(Muscles, errors)
{
    biorbd::Model model(modelPathForMuscleForce);
    biorbd::muscles::Muscles muscles;
    muscles.addMuscleGroup("muscleGroupName", "originName", "insertionName");

    EXPECT_THROW(muscles.muscleGroup(1), std::runtime_error);
    EXPECT_THROW(muscles.muscleGroup("nameNoExists"), std::runtime_error);
}

TEST(Muscles, deepCopy)
{
    biorbd::Model model(modelPathForMuscleForce);
    biorbd::muscles::Muscles muscles;
    muscles.addMuscleGroup("muscleGroupName", "originName", "insertionName");

    biorbd::muscles::Muscles shallowCopy(muscles);
    biorbd::muscles::Muscles deepCopyNow(muscles.DeepCopy());
    biorbd::muscles::Muscles deepCopyLater;
    deepCopyLater.DeepCopy(muscles);

    EXPECT_STREQ(muscles.muscleGroup(0).name().c_str(), "muscleGroupName");
    EXPECT_STREQ(shallowCopy.muscleGroup(0).name().c_str(), "muscleGroupName");
    EXPECT_STREQ(deepCopyNow.muscleGroup(0).name().c_str(), "muscleGroupName");
    EXPECT_STREQ(deepCopyLater.muscleGroup(0).name().c_str(), "muscleGroupName");

    // Set new name
    muscles.muscleGroup(0).setName("newMuscleGroupName");

    //Check if copies changed names
    EXPECT_STREQ(muscles.muscleGroup(0).name().c_str(), "newMuscleGroupName");
    EXPECT_STREQ(shallowCopy.muscleGroup(0).name().c_str(), "newMuscleGroupName");
    EXPECT_STREQ(deepCopyNow.muscleGroup(0).name().c_str(), "newMuscleGroupName");
    EXPECT_STREQ(deepCopyLater.muscleGroup(0).name().c_str(), "newMuscleGroupName");
}

TEST(WrappingHalfCylinder, unitTest)
{
    {
        biorbd::utils::RotoTrans rt(
            biorbd::utils::Vector3d(1., 1., 1.), biorbd::utils::Vector3d(1., 1., 1.),
            "xyz");

        biorbd::muscles::WrappingHalfCylinder wrappingHalfCylinder;

        wrappingHalfCylinder.setRadius(0.75);
        SCALAR_TO_DOUBLE(diameter, wrappingHalfCylinder.diameter());
        SCALAR_TO_DOUBLE(radius, wrappingHalfCylinder.radius());
        EXPECT_NEAR(diameter, 1.5, requiredPrecision);
        EXPECT_NEAR(radius, 0.75, requiredPrecision);

        wrappingHalfCylinder.setName("wrappingHalfCylinderName");
        EXPECT_STREQ(wrappingHalfCylinder.biorbd::utils::Node::name().c_str(),
                     "wrappingHalfCylinderName");
    }
    {
        biorbd::utils::RotoTrans rt(
            biorbd::utils::Vector3d(1., 1., 1.), biorbd::utils::Vector3d(1., 1., 1.),
            "xyz");

        biorbd::muscles::WrappingHalfCylinder wrappingHalfCylinder(rt, 0.25, 1.);
        SCALAR_TO_DOUBLE(diameter, wrappingHalfCylinder.diameter());
        EXPECT_NEAR(diameter, 0.5, requiredPrecision);
        SCALAR_TO_DOUBLE(length, wrappingHalfCylinder.length());
        EXPECT_NEAR(length, 1., requiredPrecision);
    }
    {
        biorbd::utils::RotoTrans rt(
            biorbd::utils::Vector3d(1., 1., 1.), biorbd::utils::Vector3d(1., 1., 1.),
            "xyz");

        biorbd::muscles::WrappingHalfCylinder wrappingHalfCylinder(rt, 0.5, 1., "name",
                "parentName");
        EXPECT_STREQ(wrappingHalfCylinder.parent().c_str(), "parentName");
    }
    {
        biorbd::muscles::WrappingHalfCylinder wrappingHalfCylinder;
        biorbd::utils::RotoTrans rt(
            biorbd::utils::Vector3d(1., 1., 1.), biorbd::utils::Vector3d(1., 1., 1.),
            "xyz");
        biorbd::utils::Vector3d p1(1., 1., 1.);
        biorbd::utils::Vector3d p2(2., 2., 2.);


        wrappingHalfCylinder.wrapPoints(
            rt,
            biorbd::utils::Vector3d(0.5, 1., 1.5),
            biorbd::utils::Vector3d(4., 5., 6.),
            p1, p2);

        SCALAR_TO_DOUBLE(p10, p1[0]);
        SCALAR_TO_DOUBLE(p11, p1[1]);
        SCALAR_TO_DOUBLE(p12, p1[2]);
        SCALAR_TO_DOUBLE(p20, p2[0]);
        SCALAR_TO_DOUBLE(p21, p2[1]);
        SCALAR_TO_DOUBLE(p22, p2[2]);
        EXPECT_NEAR(p10, 0.71229470845913756, requiredPrecision);
        EXPECT_NEAR(p11, 1.1554478324299933, requiredPrecision);
        EXPECT_NEAR(p12, 0.90018809463370408, requiredPrecision);
        EXPECT_NEAR(p20, 0.71229470845913756, requiredPrecision);
        EXPECT_NEAR(p21, 1.1554478324299933, requiredPrecision);
        EXPECT_NEAR(p22, 0.90018809463370408, requiredPrecision);
    }
}

TEST(WrappingHalfCylinder, deepCopy)
{
    biorbd::Model model(modelPathForMuscleForce);
    biorbd::utils::RotoTrans rt(
        biorbd::utils::Vector3d(1, 1, 1), biorbd::utils::Vector3d(1, 1, 1), "xyz");

    biorbd::muscles::WrappingHalfCylinder wrappingHalfCylinder(rt, 0.25, 1.);

    biorbd::muscles::WrappingHalfCylinder shallowCopy(wrappingHalfCylinder);
    biorbd::muscles::WrappingHalfCylinder deepCopyNow(
        wrappingHalfCylinder.DeepCopy());
    biorbd::muscles::WrappingHalfCylinder deepCopyLater;
    deepCopyLater.DeepCopy(wrappingHalfCylinder);

    {
        SCALAR_TO_DOUBLE(diameter, wrappingHalfCylinder.diameter());
        SCALAR_TO_DOUBLE(shallowCopyDiameter, shallowCopy.diameter());
        SCALAR_TO_DOUBLE(deepCopyNowDiameter, deepCopyNow.diameter());
        SCALAR_TO_DOUBLE(deepCopyLaterDiameter, deepCopyLater.diameter());
        EXPECT_NEAR(diameter, 0.5, requiredPrecision);
        EXPECT_NEAR(shallowCopyDiameter, 0.5, requiredPrecision);
        EXPECT_NEAR(deepCopyNowDiameter, 0.5, requiredPrecision);
        EXPECT_NEAR(deepCopyLaterDiameter, 0.5, requiredPrecision);

        SCALAR_TO_DOUBLE(length, wrappingHalfCylinder.length());
        SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.length());
        SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.length());
        SCALAR_TO_DOUBLE(deepCopyLaterlength, deepCopyLater.length());
        EXPECT_NEAR(length, 1.0, requiredPrecision);
        EXPECT_NEAR(shallowCopyLength, 1.0, requiredPrecision);
        EXPECT_NEAR(deepCopyNowLength, 1.0, requiredPrecision);
        EXPECT_NEAR(deepCopyLaterlength, 1.0, requiredPrecision);
    }

    // Set new diameter
    wrappingHalfCylinder.setRadius(1.0);
    wrappingHalfCylinder.setLength(2.5);

    // Check values
    {
        SCALAR_TO_DOUBLE(diameter, wrappingHalfCylinder.diameter());
        SCALAR_TO_DOUBLE(shallowCopyDiameter, shallowCopy.diameter());
        SCALAR_TO_DOUBLE(deepCopyNowDiameter, deepCopyNow.diameter());
        SCALAR_TO_DOUBLE(deepCopyLaterDiameter, deepCopyLater.diameter());
        EXPECT_NEAR(diameter, 2., requiredPrecision);
        EXPECT_NEAR(shallowCopyDiameter, 2., requiredPrecision);
        EXPECT_NEAR(deepCopyNowDiameter, 0.5, requiredPrecision);
        EXPECT_NEAR(deepCopyLaterDiameter, 0.5, requiredPrecision);

        SCALAR_TO_DOUBLE(length, wrappingHalfCylinder.length());
        SCALAR_TO_DOUBLE(shallowCopyLength, shallowCopy.length());
        SCALAR_TO_DOUBLE(deepCopyNowLength, deepCopyNow.length());
        SCALAR_TO_DOUBLE(deepCopyLaterlength, deepCopyLater.length());
        EXPECT_NEAR(length, 2.5, requiredPrecision);
        EXPECT_NEAR(shallowCopyLength, 2.5, requiredPrecision);
        EXPECT_NEAR(deepCopyNowLength, 1.0, requiredPrecision);
        EXPECT_NEAR(deepCopyLaterlength, 1.0, requiredPrecision);
    }
}

TEST(MuscleForce, position)
{
    // TODO
    // Position of origin, insertion, and path in local
    // Position of origin, insertion, and path in global
}

TEST(MuscleForce, force)
{
    biorbd::Model model(modelPathForMuscleForce);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity QDot(model);
    Q = Q.setOnes()/10;
    QDot = QDot.setOnes()/10;
    std::vector<std::shared_ptr<biorbd::muscles::State>> states;
    for (unsigned int i=0; i<model.nbMuscleTotal(); ++i) {
        states.push_back(std::make_shared<biorbd::muscles::StateDynamics>(0, 0.2));
    }
    model.updateMuscles(Q, QDot, true);

    const biorbd::utils::Vector& F = model.muscleForces(states);

    std::vector<double> ExpectedForce({
        164.3110575502927, 106.89637709077938, 84.340201458493794,
        92.212055754969938, 85.0882802083116, 198.6356130736217
    });
    for (unsigned int i=0; i<model.nbMuscleTotal(); ++i) {
        SCALAR_TO_DOUBLE(val, F(i));
        EXPECT_NEAR(val, ExpectedForce[i], requiredPrecision);
    }
}

TEST(MuscleForce, torqueFromMuscles)
{
    biorbd::Model model(modelPathForMuscleForce);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity QDot(model);
    biorbd::rigidbody::GeneralizedAcceleration QDDot(model);
    Q.setOnes()/10;
    QDot.setOnes()/10;
    std::vector<std::shared_ptr<biorbd::muscles::State>> states;
    for (unsigned int i=0; i<model.nbMuscleTotal(); ++i) {
        states.push_back(std::make_shared<biorbd::muscles::StateDynamics>(0, 0.2));
    }

    biorbd::rigidbody::GeneralizedTorque Tau(model);
    std::vector<double> TauExpected({-18.271389285751727, -7.820566757538376});
    Tau = model.muscularJointTorque(states, Q, QDot);
    for (unsigned int i=0; i<QDDot.size(); ++i) {
        SCALAR_TO_DOUBLE(val, Tau(i));
        EXPECT_NEAR(val, TauExpected[i], requiredPrecision);
    }

    RigidBodyDynamics::ForwardDynamics(model, Q, QDot, Tau, QDDot);
    std::vector<double> QDDotExpected({-2.4941551687243537, 0.04600953825654895});
    for (unsigned int i=0; i<QDDot.size(); ++i) {
        SCALAR_TO_DOUBLE(val, QDDot(i));
        EXPECT_NEAR(val, QDDotExpected[i], requiredPrecision);
    }
}

TEST(MuscleCharacterics, unittest)
{
    {
        biorbd::muscles::Characteristics charact;
        {
            SCALAR_TO_DOUBLE(optimalLength, charact.optimalLength());
            EXPECT_NEAR(optimalLength, 0, requiredPrecision);
        }
        double newOptimalLength(3.4);
        charact.setOptimalLength(newOptimalLength);
        {
            SCALAR_TO_DOUBLE(optimalLength, charact.optimalLength());
            EXPECT_NEAR(optimalLength, newOptimalLength, requiredPrecision);
        }
    }

    {
        biorbd::muscles::Characteristics charact;
        {
            SCALAR_TO_DOUBLE(forceIsoMax, charact.forceIsoMax());
            EXPECT_NEAR(forceIsoMax, 0, requiredPrecision);
        }
        double newForceMax(156.9);
        charact.setForceIsoMax(newForceMax);

        {
            SCALAR_TO_DOUBLE(forceIsoMax, charact.forceIsoMax());
            EXPECT_NEAR(forceIsoMax, newForceMax, requiredPrecision);
        }
    }

    {
        biorbd::muscles::Characteristics charact;
        {
            SCALAR_TO_DOUBLE(tendonSlackLength, charact.tendonSlackLength());
            EXPECT_NEAR(tendonSlackLength, 0, requiredPrecision);
        }
        double newTendonSlakLength(5.3);
        charact.setTendonSlackLength(newTendonSlakLength);
        {
            SCALAR_TO_DOUBLE(tendonSlackLength, charact.tendonSlackLength());
            EXPECT_NEAR(tendonSlackLength, newTendonSlakLength, requiredPrecision);
        }

    }

    {
        biorbd::muscles::Characteristics charact;
        {
            SCALAR_TO_DOUBLE(pennationAngle, charact.pennationAngle());
            EXPECT_NEAR(pennationAngle, 0, requiredPrecision);
        }
        double newPennationAngle(1.09);
        charact.setPennationAngle(newPennationAngle);
        {
            SCALAR_TO_DOUBLE(pennationAngle, charact.pennationAngle());
            EXPECT_NEAR(pennationAngle, newPennationAngle, requiredPrecision);
        }

    }

    {
        biorbd::muscles::Characteristics charact;
        {
            SCALAR_TO_DOUBLE(PCSA, charact.PCSA());
            EXPECT_NEAR(PCSA, 0, requiredPrecision);
        }
        double newPCSA(2.4);
        charact.setPCSA(newPCSA);
        {
            SCALAR_TO_DOUBLE(PCSA, charact.PCSA());
            EXPECT_NEAR(PCSA, newPCSA, requiredPrecision);
        }
    }
}

TEST(MuscleJacobian, jacobian)
{
    biorbd::Model model(modelPathForMuscleJacobian);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    Q = Q.setOnes()/10;

    // Force computation of geometry
    biorbd::muscles::Muscle& muscle(model.muscleGroup(
                                        muscleForMuscleJacobian).muscle(muscleGroupForMuscleJacobian));
    EXPECT_THROW(muscle.position().jacobian(), std::runtime_error);
    model.updateMuscles(Q, true);

    unsigned int nRows(3 * (muscle.pathModifier().nbObjects() + 2));
    biorbd::utils::Matrix jacoRef(nRows, model.nbQ());
    // Here we provide emperical values that we have confidence in
    jacoRef(0, 0) = 0.13689690274955996;
    jacoRef(0, 1) = 0;
    jacoRef(1, 0) = 0.0048155626519999824;
    jacoRef(1, 1) = 0;
    jacoRef(2, 0) = 0.0080659044260533528;
    jacoRef(2, 1) = 0;
    jacoRef(3, 0) = 0.1530001664829288;
    jacoRef(3, 1) = 0;
    jacoRef(4, 0) = -0.011356405676791911;
    jacoRef(4, 1) = 0;
    jacoRef(5, 0) = 0.0090532669706573556;
    jacoRef(5, 1) = 0;
    jacoRef(6, 0) = 0.22806179660848588;
    jacoRef(6, 1) = 0;
    jacoRef(7, 0) = -0.0097422466736260017;
    jacoRef(7, 1) = 0;
    jacoRef(8, 0) = 0.013478229213671932;
    jacoRef(8, 1) = 0;
    jacoRef(9, 0) = 0.2675513197235147;
    jacoRef(9, 1) = 0;
    jacoRef(10, 0) = 0.0086890497375387721;
    jacoRef(10, 1) = 0;
    jacoRef(11, 0) = 0.015765668988435677;
    jacoRef(11, 1) = 0;
    jacoRef(12, 0) = 0.28198829726211816;
    jacoRef(12, 1) = -0.0059281232164749703;
    jacoRef(13, 0) = 0.010784640478103213;
    jacoRef(13, 1) = -0.023460590547529164;
    jacoRef(14, 0) = 0.016612631083717935;
    jacoRef(14, 1) = 0.0013801799073302687;

    // Compare with computed values
    biorbd::utils::Matrix jaco(muscle.position().jacobian());
    for (unsigned int i=0; i<jaco.rows(); ++i) {
        for (unsigned int j=0; j<jaco.cols(); ++j) {
            SCALAR_TO_DOUBLE(val, jaco(i, j));
            SCALAR_TO_DOUBLE(valRef, jacoRef(i, j));
            EXPECT_NEAR(val, valRef, requiredPrecision);
        }
    }

    // Change Q
    Q.setOnes();
    model.updateMuscles(Q, true);
    // Here we provide emperical values that we have confidence in
    jacoRef(0, 0) = 0.08134540996216065;
    jacoRef(0, 1) = 0;
    jacoRef(1, 0) = 0.11041411771499073;
    jacoRef(1, 1) = 0;
    jacoRef(2, 0) = 0.0045450332887922848;
    jacoRef(2, 1) = 0;
    jacoRef(3, 0) = 0.10400305103312649;
    jacoRef(3, 1) = 0;
    jacoRef(4, 0) = 0.1129992026935549;
    jacoRef(4, 1) = 0;
    jacoRef(5, 0) = 0.0058758916739148634;
    jacoRef(5, 1) = 0;
    jacoRef(6, 0) = 0.14940786497893349;
    jacoRef(6, 1) = 0;
    jacoRef(7, 0) = 0.17290229814238411;
    jacoRef(7, 1) = 0;
    jacoRef(8, 0) = 0.008416785894586807;
    jacoRef(8, 1) = 0;
    jacoRef(9, 0) = 0.1595464389865561;
    jacoRef(9, 1) = 0;
    jacoRef(10, 0) = 0.21534433421098692;
    jacoRef(10, 1) = 0;
    jacoRef(11, 0) = 0.0089171811535144387;
    jacoRef(11, 1) = 0;
    jacoRef(12, 0) = 0.17634886417858722;
    jacoRef(12, 1) = 0.024220863439657705;
    jacoRef(13, 0) = 0.24680272575831863;
    jacoRef(13, 1) = -0.00046752133823085851;
    jacoRef(14, 0) = 0.0098360540170798361;
    jacoRef(14, 1) = 0.00076029489073317982;

    // Compare with computed values
    jaco = muscle.position().jacobian();
    for (unsigned int i=0; i<jaco.rows(); ++i) {
        for (unsigned int j=0; j<jaco.cols(); ++j) {
            SCALAR_TO_DOUBLE(val, jaco(i, j));
            SCALAR_TO_DOUBLE(valRef, jacoRef(i, j));
            EXPECT_NEAR(val, valRef, requiredPrecision);
        }
    }
}

TEST(MuscleJacobian, jacobianLength)
{
    biorbd::Model model(modelPathForMuscleJacobian);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    Q = Q.setOnes()/10;
    model.updateMuscles(Q, true);

    unsigned int nRows(model.nbMuscleTotal());
    biorbd::utils::Matrix jacoRef(nRows, model.nbQ());
    // Here we provide emperical values that we have confidence in
    jacoRef(0, 0) = 0.037620360527045288;
    jacoRef(0, 1) = 0.022230374109936522;
    jacoRef(1, 0) = -0.017006708057341259;
    jacoRef(1, 1) = -0.012779004692822197;
    jacoRef(2, 0) = -0.0016529882883160136;
    jacoRef(2, 1) = -0.012779004692822197;
    jacoRef(3, 0) =  2.6526502766149064e-17;
    jacoRef(3, 1) = 0.022230374109936522;
    jacoRef(4, 0) =  5.2979083494670027e-18;
    jacoRef(4, 1) = 0.022230374109936522;
    jacoRef(5, 0) =  5.3888967663972615e-18;
    jacoRef(5, 1) = 0.00064415763125857495;

    // Compare with computed values
    biorbd::utils::Matrix jaco(model.musclesLengthJacobian(Q));
    for (unsigned int i=0; i<jaco.rows(); ++i) {
        for (unsigned int j=0; j<jaco.cols(); ++j) {
            SCALAR_TO_DOUBLE(val, jaco(i, j));
            SCALAR_TO_DOUBLE(valRef, jacoRef(i, j));
            EXPECT_NEAR(val, valRef, requiredPrecision);
        }
    }

    // Change Q
    Q.setOnes();
    model.updateMuscles(Q, true);
    // Here we provide emperical values that we have confidence in
    jacoRef(0, 0) = 0.025994784772561841;
    jacoRef(0, 1) = 0.021586396946686921;
    jacoRef(1, 0) = -0.011088003983559729;
    jacoRef(1, 1) = -0.039462418895871744;
    jacoRef(2, 0) = 0.032285277521606391;
    jacoRef(2, 1) = -0.039462418895871744;
    jacoRef(3, 0) =  1.1900679209044559e-19;
    jacoRef(3, 1) = 0.021586396946686921;
    jacoRef(4, 0) =  1.4848744796707674e-17;
    jacoRef(4, 1) = 0.021586396946686921;
    jacoRef(5, 0) =  -2.5088055592238924e-17;
    jacoRef(5, 1) = -0.015991501107813871;
    jaco = model.musclesLengthJacobian(Q);
    for (unsigned int i=0; i<jaco.rows(); ++i) {
        for (unsigned int j=0; j<jaco.cols(); ++j) {
            SCALAR_TO_DOUBLE(val, jaco(i, j));
            SCALAR_TO_DOUBLE(valRef, jacoRef(i, j));
            EXPECT_NEAR(val, valRef, requiredPrecision);
        }
    }
}

#ifndef BIORBD_USE_CASADI_MATH
TEST(MuscleFatigue, FatigueXiaDerivativeViaPointers)
{
    // Prepare the model
    biorbd::Model model(modelPathForXiaDerivativeTest);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity QDot(model);
    Q.setZero();
    QDot.setZero();
    model.updateMuscles(Q, QDot, true);

    {
        biorbd::muscles::HillThelenTypeFatigable muscle(model.muscleGroup(
                    muscleGroupForXiaDerivativeTest).muscle(
                    muscleForXiaDerivativeTest));
        biorbd::muscles::FatigueDynamicState& fatigueModel(
            dynamic_cast<biorbd::muscles::FatigueDynamicState&>
            (muscle.fatigueState()));
        // Sanity check for the fatigue model
        EXPECT_EQ(fatigueModel.getType(),
                  biorbd::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

        // Initial value sanity check
        {
            SCALAR_TO_DOUBLE(activeFibersDot, fatigueModel.activeFibersDot());
            SCALAR_TO_DOUBLE(fatiguedFibersDot, fatigueModel.fatiguedFibersDot());
            SCALAR_TO_DOUBLE(restingFibersDot, fatigueModel.restingFibersDot());
            EXPECT_NEAR(activeFibersDot, 0, requiredPrecision);
            EXPECT_NEAR(fatiguedFibersDot, 0, requiredPrecision);
            EXPECT_NEAR(restingFibersDot, 0, requiredPrecision);
        }

        // Apply the derivative
        biorbd::muscles::StateDynamics emg(0,
                                           activationEmgForXiaDerivativeTest); // Set target
        fatigueModel.setState(currentActiveFibersForXiaDerivativeTest,
                              currentFatiguedFibersForXiaDerivativeTest,
                              currentRestingFibersForXiaDerivativeTest);
        fatigueModel.timeDerivativeState(emg,
                                         model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(
                                             muscleForXiaDerivativeTest).characteristics());

        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibersDot(),
                    expectedActivationDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibersDot(),
                    expectedFatiguedDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibersDot(),
                    expectedRestingDotForXiaDerivativeTest, requiredPrecision);
    }

    // Values should be changed in the model itself
    {
        biorbd::muscles::HillThelenTypeFatigable muscle(model.muscleGroup(
                    muscleGroupForXiaDerivativeTest).muscle(
                    muscleForXiaDerivativeTest));
        biorbd::muscles::FatigueDynamicState& fatigueModel(
            dynamic_cast<biorbd::muscles::FatigueDynamicState&>
            (muscle.fatigueState()));

        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibersDot(),
                    expectedActivationDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibersDot(),
                    expectedFatiguedDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibersDot(),
                    expectedRestingDotForXiaDerivativeTest, requiredPrecision);
    }
}
#endif

#ifndef BIORBD_USE_CASADI_MATH
TEST(MuscleFatigue, FatigueXiaDerivativeViaInterface)
{
    // Prepare the model
    biorbd::Model model(modelPathForXiaDerivativeTest);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity QDot(model);
    Q.setZero();
    QDot.setZero();
    model.updateMuscles(Q, QDot, true);

    {
        biorbd::muscles::HillThelenTypeFatigable muscle(model.muscleGroup(
                    muscleGroupForXiaDerivativeTest).muscle(
                    muscleForXiaDerivativeTest));

        // Apply the derivative
        biorbd::muscles::StateDynamics emg(0,
                                           activationEmgForXiaDerivativeTest); // Set target
        muscle.setFatigueState(currentActiveFibersForXiaDerivativeTest,
                               currentFatiguedFibersForXiaDerivativeTest,
                               currentRestingFibersForXiaDerivativeTest);
        muscle.computeTimeDerivativeState(emg);

    }

    // Values should be changed in the model itself
    {
        biorbd::muscles::HillThelenTypeFatigable muscle(model.muscleGroup(0).muscle(0));
        biorbd::muscles::FatigueDynamicState& fatigueModel(
            dynamic_cast<biorbd::muscles::FatigueDynamicState&>
            (muscle.fatigueState()));

        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibersDot(),
                    expectedActivationDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibersDot(),
                    expectedFatiguedDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibersDot(),
                    expectedRestingDotForXiaDerivativeTest, requiredPrecision);
    }
}
#endif

#ifndef BIORBD_USE_CASADI_MATH
TEST(MuscleFatigue, FatigueXiaDerivativeShallowViaCopy)
{
    // Prepare the model
    biorbd::Model model(modelPathForXiaDerivativeTest);
    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity QDot(model);
    Q.setZero();
    QDot.setZero();
    model.updateMuscles(Q, QDot, true);

    {
        biorbd::muscles::HillThelenTypeFatigable muscle(
            model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(
                muscleForXiaDerivativeTest));
        biorbd::muscles::FatigueDynamicStateXia fatigueModel(
            dynamic_cast<biorbd::muscles::FatigueDynamicStateXia&>
            (muscle.fatigueState()));
        // Sanity check for the fatigue model
        EXPECT_EQ(fatigueModel.getType(),
                  biorbd::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

        // Initial value sanity check
        {
            SCALAR_TO_DOUBLE(activeFibersDot, fatigueModel.activeFibersDot());
            SCALAR_TO_DOUBLE(fatiguedFibersDot, fatigueModel.fatiguedFibersDot());
            SCALAR_TO_DOUBLE(restingFibersDot, fatigueModel.restingFibersDot());
            EXPECT_NEAR(activeFibersDot, 0, requiredPrecision);
            EXPECT_NEAR(fatiguedFibersDot, 0, requiredPrecision);
            EXPECT_NEAR(restingFibersDot, 0, requiredPrecision);
        }

        // Apply the derivative
        biorbd::muscles::StateDynamics emg(0,
                                           activationEmgForXiaDerivativeTest); // Set target
        fatigueModel.setState(currentActiveFibersForXiaDerivativeTest,
                              currentFatiguedFibersForXiaDerivativeTest,
                              currentRestingFibersForXiaDerivativeTest);
        fatigueModel.timeDerivativeState(emg,
                                         model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(
                                             muscleForXiaDerivativeTest).characteristics());

        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibersDot(),
                    expectedActivationDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibersDot(),
                    expectedFatiguedDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibersDot(),
                    expectedRestingDotForXiaDerivativeTest, requiredPrecision);
    }

    // Values should be changed in the model itself since everything is shallowcopied
    {
        biorbd::muscles::HillThelenTypeFatigable muscle(model.muscleGroup(
                    muscleGroupForXiaDerivativeTest).muscle(
                    muscleForXiaDerivativeTest));
        biorbd::muscles::FatigueDynamicState& fatigueModel(
            dynamic_cast<biorbd::muscles::FatigueDynamicState&>
            (muscle.fatigueState()));

        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibersDot(),
                    expectedActivationDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibersDot(),
                    expectedFatiguedDotForXiaDerivativeTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibersDot(),
                    expectedRestingDotForXiaDerivativeTest, requiredPrecision);
    }
}
#endif

#ifndef BIORBD_USE_CASADI_MATH
TEST(MuscleFatigue, FatigueXiaSetStateLimitsTest)
{
    // Prepare the model
    biorbd::Model model(modelPathForXiaDerivativeTest);
    biorbd::muscles::HillThelenTypeFatigable muscle(
        model.muscleGroup(muscleGroupForXiaDerivativeTest).muscle(
            muscleForXiaDerivativeTest));
    biorbd::muscles::FatigueDynamicStateXia fatigueModel(
        dynamic_cast<biorbd::muscles::FatigueDynamicStateXia&>
        (muscle.fatigueState()));
    // Sanity check for the fatigue model
    EXPECT_EQ(fatigueModel.getType(),
              biorbd::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA);

    // Teast limit active quantity
    {
        // Set values under 0
        fatigueModel.setState(negativeFibersQuantityForFatigueXiaSetStateLimitsTest,
                              positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              positiveFibersQuantityForFatigueXiaSetStateLimitsTest);


        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibers(), 0, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibers(),
                    positiveFibersQuantityForFatigueXiaSetStateLimitsTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibers(),
                    positiveFibersQuantityForFatigueXiaSetStateLimitsTest
                    +negativeFibersQuantityForFatigueXiaSetStateLimitsTest,
                    requiredPrecision);

        // Set values over 1
        fatigueModel.setState(excessiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              0,
                              positiveFibersQuantityForFatigueXiaSetStateLimitsTest
                              -excessiveFibersQuantityForFatigueXiaSetStateLimitsTest);


        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibers(),
                    positiveFibersQuantityForFatigueXiaSetStateLimitsTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibers(), 0, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibers(), 0, requiredPrecision);

    }

    //Test limit resting quantity
    {
        // Set values to be corrected
        fatigueModel.setState(positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                              negativeFibersQuantityForFatigueXiaSetStateLimitsTest);

        // Check the values
        EXPECT_NEAR(fatigueModel.activeFibers(),
                    positiveFibersQuantityForFatigueXiaSetStateLimitsTest
                    +negativeFibersQuantityForFatigueXiaSetStateLimitsTest,
                    requiredPrecision);
        EXPECT_NEAR(fatigueModel.fatiguedFibers(),
                    positiveFibersQuantityForFatigueXiaSetStateLimitsTest, requiredPrecision);
        EXPECT_NEAR(fatigueModel.restingFibers(), 0, requiredPrecision);

        // Set incorrect values
        EXPECT_THROW(fatigueModel.setState(
                         positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                         positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                         excessiveFibersQuantityForFatigueXiaSetStateLimitsTest), std::runtime_error);
    }

    // Test limit fatigued quantity
    {
        // Set values
        EXPECT_THROW(fatigueModel.setState(
                         positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                         negativeFibersQuantityForFatigueXiaSetStateLimitsTest,
                         positiveFibersQuantityForFatigueXiaSetStateLimitsTest), std::runtime_error);
        EXPECT_THROW(fatigueModel.setState(
                         positiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                         excessiveFibersQuantityForFatigueXiaSetStateLimitsTest,
                         positiveFibersQuantityForFatigueXiaSetStateLimitsTest), std::runtime_error);

    }
}
#endif

#ifdef MODULE_STATIC_OPTIM

TEST(StaticOptim, OneFrameNoActivations)
{
#ifdef BIORBD_USE_CASADI_MATH
    std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
    biorbd::Model model(modelPathForMuscleForce);

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    for (unsigned int i=0; i<Q.size(); ++i) {
        Q[i] = static_cast<double>(i) * 1.1;
        Qdot[i] = static_cast<double>(i) * 1.1;
        Tau[i] = static_cast<double>(i) * 1.1;
    }

    // Proceed with the static optimization
    auto optim = biorbd::muscles::StaticOptimization(model, Q, Qdot, Tau);
    optim.run();
    auto muscleActivations = optim.finalSolution()[0];

    std::vector<double> expectedActivations = {
        0.00010045072897390454, 0.00023334006766472334, 0.00010325993967600416,
        0.00033780547738511266,  0.00032642282294118751, 0.00010173561179265281
    };
    for (size_t i=0; i<expectedActivations.size(); ++i) {
        EXPECT_NEAR(muscleActivations(i), expectedActivations[i], 1e-5);
    }

#endif
}

TEST(StaticOptim, OneFrameOneActivationDouble)
{
#ifdef BIORBD_USE_CASADI_MATH
    std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
    biorbd::Model model(modelPathForMuscleForce);

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    for (unsigned int i=0; i<Q.size(); ++i) {
        Q[i] = static_cast<double>(i) * 1.1;
        Qdot[i] = static_cast<double>(i) * 1.1;
        Tau[i] = static_cast<double>(i) * 1.1;
    }

    // Proceed with the static optimization
    double initialActivationGuess = 0.5;
    auto optim = biorbd::muscles::StaticOptimization(model, Q, Qdot, Tau,
                 initialActivationGuess);
    optim.run();
    auto muscleActivations = optim.finalSolution()[0];

    std::vector<double> expectedActivations = {
        0.00010045072897390454, 0.00023334006766472334, 0.00010325993967600416,
        0.00033780547738511266,  0.00032642282294118751, 0.00010173561179265281
    };
    for (size_t i=0; i<expectedActivations.size(); ++i) {
        EXPECT_NEAR(muscleActivations(i), expectedActivations[i], 1e-5);
    }

#endif
}

TEST(StaticOptim, OneFrameOneActivationVector)
{
#ifdef BIORBD_USE_CASADI_MATH
    std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
    biorbd::Model model(modelPathForMuscleForce);

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    for (unsigned int i=0; i<Q.size(); ++i) {
        Q[i] = static_cast<double>(i) * 1.1;
        Qdot[i] = static_cast<double>(i) * 1.1;
        Tau[i] = static_cast<double>(i) * 1.1;
    }

    // Proceed with the static optimization
    biorbd::utils::Vector initialActivationGuess(model.nbMuscles());
    for (unsigned int i=0; i<model.nbMuscles(); ++i) {
        initialActivationGuess[i] = 0.5;
    }
    auto optim = biorbd::muscles::StaticOptimization(model, Q, Qdot, Tau,
                 initialActivationGuess);
    optim.run();
    auto muscleActivations = optim.finalSolution()[0];

    std::vector<double> expectedActivations = {
        0.00010045072897390454, 0.00023334006766472334, 0.00010325993967600416,
        0.00033780547738511266,  0.00032642282294118751, 0.00010173561179265281
    };
    for (size_t i=0; i<expectedActivations.size(); ++i) {
        EXPECT_NEAR(muscleActivations(i), expectedActivations[i], 1e-5);
    }

#endif
}

TEST(StaticOptim, MultiFrameNoActivation)
{
#ifdef BIORBD_USE_CASADI_MATH
    std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
    biorbd::Model model(modelPathForMuscleForce);

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    for (unsigned int i=0; i<Q.size(); ++i) {
        Q[i] = static_cast<double>(i) * 1.1;
        Qdot[i] = static_cast<double>(i) * 1.1;
        Tau[i] = static_cast<double>(i) * 1.1;
    }
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> allQ;
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> allQdot;
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> allTau;
    allQ.push_back(Q);
    allQ.push_back(Q);
    allQ.push_back(Q);
    allQdot.push_back(Qdot);
    allQdot.push_back(Qdot);
    allQdot.push_back(Qdot);
    allTau.push_back(Tau);
    allTau.push_back(Tau);
    allTau.push_back(Tau);

    // Proceed with the static optimization
    auto optim = biorbd::muscles::StaticOptimization(model, Q, Qdot, Tau);
    optim.run();
    auto allMuscleActivations = optim.finalSolution();

    std::vector<double> expectedActivations = {
        0.00010045072897390454, 0.00023334006766472334, 0.00010325993967600416,
        0.00033780547738511266,  0.00032642282294118751, 0.00010173561179265281
    };
    for (auto muscleActivations : allMuscleActivations) {
        for (size_t i=0; i<expectedActivations.size(); ++i) {
            EXPECT_NEAR(muscleActivations(i), expectedActivations[i], 1e-5);
        }
    }

#endif
}

TEST(StaticOptim, MultiFrameActivationDouble)
{
#ifdef BIORBD_USE_CASADI_MATH
    std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
    biorbd::Model model(modelPathForMuscleForce);

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    for (unsigned int i=0; i<Q.size(); ++i) {
        Q[i] = static_cast<double>(i) * 1.1;
        Qdot[i] = static_cast<double>(i) * 1.1;
        Tau[i] = static_cast<double>(i) * 1.1;
    }
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> allQ;
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> allQdot;
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> allTau;
    allQ.push_back(Q);
    allQ.push_back(Q);
    allQ.push_back(Q);
    allQdot.push_back(Qdot);
    allQdot.push_back(Qdot);
    allQdot.push_back(Qdot);
    allTau.push_back(Tau);
    allTau.push_back(Tau);
    allTau.push_back(Tau);

    // Proceed with the static optimization
    double initialActivationGuess = 0.5;
    auto optim = biorbd::muscles::StaticOptimization(model, Q, Qdot, Tau,
                 initialActivationGuess);
    optim.run();
    auto allMuscleActivations = optim.finalSolution();

    std::vector<double> expectedActivations = {
        0.00010045072897390454, 0.00023334006766472334, 0.00010325993967600416,
        0.00033780547738511266,  0.00032642282294118751, 0.00010173561179265281
    };
    for (auto muscleActivations : allMuscleActivations) {
        for (size_t i=0; i<expectedActivations.size(); ++i) {
            EXPECT_NEAR(muscleActivations(i), expectedActivations[i], 1e-5);
        }
    }

#endif
}

TEST(StaticOptim, MultiFrameNoActivationVector)
{
#ifdef BIORBD_USE_CASADI_MATH
    std::cout << "StaticOptim is not tested for CasADi backend" << std::endl;

#else
    biorbd::Model model(modelPathForMuscleForce);

    biorbd::rigidbody::GeneralizedCoordinates Q(model);
    biorbd::rigidbody::GeneralizedVelocity Qdot(model);
    biorbd::rigidbody::GeneralizedTorque Tau(model);
    for (unsigned int i=0; i<Q.size(); ++i) {
        Q[i] = static_cast<double>(i) * 1.1;
        Qdot[i] = static_cast<double>(i) * 1.1;
        Tau[i] = static_cast<double>(i) * 1.1;
    }
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> allQ;
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> allQdot;
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> allTau;
    allQ.push_back(Q);
    allQ.push_back(Q);
    allQ.push_back(Q);
    allQdot.push_back(Qdot);
    allQdot.push_back(Qdot);
    allQdot.push_back(Qdot);
    allTau.push_back(Tau);
    allTau.push_back(Tau);
    allTau.push_back(Tau);

    // Proceed with the static optimization
    biorbd::utils::Vector initialActivationGuess(model.nbMuscles());
    for (unsigned int i=0; i<model.nbMuscles(); ++i) {
        initialActivationGuess[i] = 0.5;
    }
    auto optim = biorbd::muscles::StaticOptimization(model, Q, Qdot, Tau,
                 initialActivationGuess);
    optim.run();
    auto allMuscleActivations = optim.finalSolution();

    std::vector<double> expectedActivations = {
        0.00010045072897390454, 0.00023334006766472334, 0.00010325993967600416,
        0.00033780547738511266,  0.00032642282294118751, 0.00010173561179265281
    };
    for (auto muscleActivations : allMuscleActivations) {
        for (size_t i=0; i<expectedActivations.size(); ++i) {
            EXPECT_NEAR(muscleActivations(i), expectedActivations[i], 1e-5);
        }
    }

#endif
}




#endif

#endif // MODULE_MUSCLES
