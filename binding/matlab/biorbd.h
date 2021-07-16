#ifndef MATLAB_BIORBD_H
#define MATLAB_BIORBD_H

#include <mex.h>
#include <cmath>
#include <iostream>
#include <algorithm>
#include "Utils/Error.h"

#include "Matlab_help.h"
#include "Matlab_new.h"
#include "Matlab_delete.h"
#include "Matlab_parent.h"
#include "Matlab_changeGravity.h"
#include "Matlab_totalMass.h"
#include "Matlab_massMatrix.h"
#include "Matlab_nControl.h"
#include "Matlab_nQ.h"
#include "Matlab_nQdot.h"
#include "Matlab_nQddot.h"
#include "Matlab_rangeQ.h"
#include "Matlab_nameDof.h"
#include "Matlab_nRoot.h"
#include "Matlab_nMarkers.h"
#include "Matlab_nameMarkers.h"
#include "Matlab_nBody.h"
#include "Matlab_nameBody.h"
#include "Matlab_segmentMass.h"
#include "Matlab_Markers.h"
#include "Matlab_ProjectCustomPoint.h"
#include "Matlab_ProjectPoint.h"
#include "Matlab_ProjectPointJacobian.h"
#include "Matlab_nIMU.h"
#include "Matlab_nameIMU.h"
#include "Matlab_IMU.h"
#include "Matlab_IMUJacobian.h"
#include "Matlab_LocalMarkers.h"
#include "Matlab_localJCS.h"
#include "Matlab_segmentMarkers.h"
#include "Matlab_inverseKinematics.h"
#ifdef MODULE_KALMAN
    #include "Matlab_inverseKinematicsEKF.h"
    #include "Matlab_inverseKinematicsEKF_IMU.h"
#endif
#include "Matlab_NLeffects.h"
#include "Matlab_inverseDynamics.h"
#include "Matlab_forwardDynamics.h"
#include "Matlab_computeQdot.h"
#include "Matlab_Mesh.h"
#include "Matlab_Patch.h"
#include "Matlab_MarkersJacobian.h"
#include "Matlab_ContactsPosition.h"
#include "Matlab_ContactJacobian.h"
#include "Matlab_ContactGamma.h"
#include "Matlab_CoM.h"
#include "Matlab_CoMJacobian.h"
#include "Matlab_CoMdot.h"
#include "Matlab_CoMddot.h"
#include "Matlab_segmentCoM.h"
#include "Matlab_segmentCoMdot.h"
#include "Matlab_segmentCoMddot.h"
#include "Matlab_CoMangularMomentum.h"
#include "Matlab_segmentAngularMomentum.h"
#include "Matlab_segmentsInertia.h"
#include "Matlab_segmentsVelocities.h"
#include "Matlab_globalJCS.h"

#ifdef MODULE_MUSCLES
    #include "Matlab_muscleUpdate.h"
    #include "Matlab_MusclesPoints.h"
    #include "Matlab_MusclesJacobian.h"
    #include "Matlab_MusclesLength.h"
    #include "Matlab_muscleLengthJacobian.h"
    #include "Matlab_muscleVelocity.h"
    #include "Matlab_MusclesNames.h"
    #include "Matlab_MusclesParentNames.h"
    #include "Matlab_nMuscles.h"
    #include "Matlab_muscleJointTorqueFromActivation.h"
    #include "Matlab_muscleJointTorqueFromExcitation.h"
    #include "Matlab_muscleJointTorqueFromMuscleForce.h"
    #include "Matlab_MusclesForce.h"
    #include "Matlab_MusclesForceMax.h"
    #include "Matlab_MusclesActivationDot.h"
    #include "Matlab_MusclesExcitationDotBuchanan.h"
    #include "Matlab_ChangeShapeFactors.h"
#endif // MODULE_MUSCLES

#ifdef MODULE_ACTUATORS
    #include "Matlab_torqueActivation.h"
#endif // MODULE_ACTUATORS

std::string toLower(const std::string &str)
{
    std::string new_str = str;
    std::transform(new_str.begin(), new_str.end(), new_str.begin(), ::tolower);
    return new_str;
}

// MATLAB INTERFACE
///
/// \brief Entry point for the muscod application
///
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] );
void functionHub( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] )
{
    // Check for proper number of arguments
    if (nrhs < 1) {
        mexErrMsgIdAndTxt( "MATLAB:yprime:invalidNumInputs",
                           "First argument should be the command");
    }

    // Traitement du 1er argument qui est la commande
    // Check for proper input type
    if (!mxIsChar(prhs[0]) || (mxGetM(prhs[0]) != 1 ) )  {
        mexErrMsgIdAndTxt( "MATLAB:mxmalloc:invalidInput",
                           "Input argument 1 must be a command string.");
    }
    char *cmd_char = mxArrayToString(prhs[0]);
    std::string cmd(cmd_char);
    mxFree(cmd_char);

    // Si on a demandé de l'aide
    if (!toLower(cmd).compare("help")) { //!strcmp("help", cmd)){
        Matlab_help();
        return;
    }


    // Redirect vers les bonnes fonctions

    // À l'appel d'un nouveau modèle
    if (!toLower(cmd).compare("new")) {
        Matlab_new(nlhs, plhs, nrhs, prhs);
        return;
    }

    // À l'appel du delete du modèle
    if (!toLower(cmd).compare("delete")) {
        Matlab_delete(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Changer la gravité du modèle
    if (!toLower(cmd).compare("gravity")) {
        Matlab_changeGravity(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de GeneralizedTorque
    if (!toLower(cmd).compare("ngeneralizedtorque")
            || !toLower(cmd).compare("ntau")) {
        Matlab_nGeneralizedTorque(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de dof sur le segment racine
    if (!toLower(cmd).compare("nroot")) {
        Matlab_nRoot(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de dof sur le segment racine
    if (!toLower(cmd).compare("parent")) {
        Matlab_parent(nlhs, plhs, nrhs, prhs);
        return;
    }


    // mass totale du systeme
    if (!toLower(cmd).compare("totalmass")) {
        Matlab_totalMass(nlhs, plhs, nrhs, prhs);
        return;
    }

    // matrice de masse en fonction de la position
    if(!toLower(cmd).compare("massmatrix")) {
        Matlab_massMatrix(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de dof
    if (!toLower(cmd).compare("nq")) {
        Matlab_nQ(nlhs, plhs, nrhs, prhs);
        return;
    }
    if (!toLower(cmd).compare("rangeq")) {
        Matlab_rangeQ(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de qdot
    if (!toLower(cmd).compare("nqdot")) {
        Matlab_nQdot(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Nombre de dof
    if (!toLower(cmd).compare("nqddot")) {
        Matlab_nQddot(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Nombre de dof
    if (!toLower(cmd).compare("namedof")) {
        Matlab_nameDof(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de Markers
    if (!toLower(cmd).compare("nmarkers")) {
        Matlab_nMarkers(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Noms des Markers
    if (!toLower(cmd).compare("namemarkers")) {
        Matlab_nameMarkers(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Noms des Markers techniques
    if (!toLower(cmd).compare("nametechnicalmarkers")) {
        Matlab_nameTechnicalMarkers(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Noms des Markers anatomiques
    if (!toLower(cmd).compare("nameanatomicalmarkers")) {
        Matlab_nameAnatomicalMarkers(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Local Markers
    if (!toLower(cmd).compare("localmarkers")) {
        Matlab_LocalMarkers(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de segments
    if (!toLower(cmd).compare("nbody")) {
        Matlab_nBody(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Noms des segments
    if (!toLower(cmd).compare("namebody")) {
        Matlab_nameBody(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Masse des segments
    if (!toLower(cmd).compare("segmentmass")) {
        Matlab_segmentMass(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique directe
    if(!toLower(cmd).compare("markers")) {
        Matlab_Markers(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique directe
    if(!toLower(cmd).compare("projectpoint")) {
        Matlab_ProjectPoint(nlhs, plhs, nrhs, prhs);
        return;
    }
    // Fonction de cinématique directe
    if(!toLower(cmd).compare("projectpointjacobian")) {
        Matlab_ProjectPointJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique directe
    if(!toLower(cmd).compare("projectcustompoint")) {
        Matlab_ProjectCustomPoint(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Nombre de Markers
    if (!toLower(cmd).compare("nmimu")) {
        biorbd::utils::Error::warning(0,
                                      "La fonction \"nmimu\" est obsolete. Remplacer par \"nimu\". Elle sera retirée prochainement");
        Matlab_nIMU(nlhs, plhs, nrhs, prhs);
        return;
    }
    if (!toLower(cmd).compare("nimu")) {
        Matlab_nIMU(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Noms des Markers
    if (!toLower(cmd).compare("namemimu")) {
        biorbd::utils::Error::warning(0,
                                      "La fonction \"namemimu\" est obsolete. Remplacer par \"nameimu\". Elle sera retirée prochainement");
        Matlab_nameIMU(nlhs, plhs, nrhs, prhs);
        return;
    }
    if (!toLower(cmd).compare("nameimu")) {
        Matlab_nameIMU(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Noms des Markers techniques
    if (!toLower(cmd).compare("nametechnicalmimu")) {
        biorbd::utils::Error::warning(0,
                                      "La fonction \"nametechnicalmimu\" est obsolete. Remplacer par \"nametechnicalimu\". Elle sera retirée prochainement");
        Matlab_nameTechnicalIMU(nlhs, plhs, nrhs, prhs);
        return;
    }
    if (!toLower(cmd).compare("nametechnicalimu")) {
        Matlab_nameTechnicalIMU(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Noms des Markers anatomiques
    if (!toLower(cmd).compare("nameanatomicalmimu")) {
        biorbd::utils::Error::warning(0,
                                      "La fonction \"nameanatomicalmimu\" est obsolete. Remplacer par \"nameanatomicalimu\". Elle sera retirée prochainement");
        Matlab_nameAnatomicalIMU(nlhs, plhs, nrhs, prhs);
        return;
    }
    if (!toLower(cmd).compare("nameanatomicalmimu")) {
        Matlab_nameAnatomicalIMU(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique directe
    if(!toLower(cmd).compare("mimu")) {
        biorbd::utils::Error::warning(0,
                                      "La fonction \"mimu\" est obsolete. Remplacer par \"imu\". Elle sera retirée prochainement");
        Matlab_IMU(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("imu")) {
        Matlab_IMU(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique directe
    if(!toLower(cmd).compare("mimujacobian")) {
        biorbd::utils::Error::warning(0,
                                      "La fonction \"mimujacobian\" est obsolete. Remplacer par \"imujacobian\". Elle sera retirée prochainement");
        Matlab_IMUJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("imujacobian")) {
        Matlab_IMUJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }


    // Fonction de cinématique inverse
    if(!toLower(cmd).compare("ik")) {
        Matlab_inverseKinematics(nlhs, plhs, nrhs, prhs);
        return;
    }

#ifdef MODULE_KALMAN
    // Fonction de cinématique inverse
    if(!toLower(cmd).compare("ik_ekf")) {
        Matlab_inverseKinematicsEKFallInOneCall(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique inverse par filtre de kalman
    if(!toLower(cmd).compare("ik_ekf_new")) {
        Matlab_setEKF(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("ik_ekf_step")) {
        Matlab_inverseKinematicsEKFstep(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("ik_ekf_delete")) {
        Matlab_delEKF(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique inverse
    if(!toLower(cmd).compare("ik_ekf_imu")) {
        Matlab_inverseKinematicsEKF_IMUallInOneCall(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique inverse par filtre de kalman
    if(!toLower(cmd).compare("ik_ekf_imu_new")) {
        Matlab_setEKF_IMU(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("ik_ekf_imu_step")) {
        Matlab_inverseKinematicsEKF_IMUstep(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("ik_ekf_imu_delete")) {
        Matlab_delEKF_IMU(nlhs, plhs, nrhs, prhs);
        return;
    }
#endif

    // Fonction de cinématique directe
    if(!toLower(cmd).compare("segmentsmarkers")) {
        Matlab_segmentMarkers(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de la Jacobienne de la cinématique directe
    if(!toLower(cmd).compare("markersjacobian")) {
        Matlab_MarkersJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de cinématique directe pour les points de contacts
    if(!toLower(cmd).compare("contacts")) {
        Matlab_ContactsPosition(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("contactjacobian")) {
        Matlab_ContactJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("contactgamma")) {
        Matlab_ContactGamma(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver le Centre de masse
    if(!toLower(cmd).compare("com")) {
        Matlab_CoM(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour obtenir la jacobienne du centre de masse
    if(!toLower(cmd).compare("comjacobian")) {
        Matlab_CoMJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver la vitesse du Centre de masse
    if(!toLower(cmd).compare("comdot")) {
        Matlab_CoMdot(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver l'acceleration du Centre de masse
    if(!toLower(cmd).compare("comddot")) {
        Matlab_CoMddot(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver l'acceleration du Centre de masse
    if(!toLower(cmd).compare("mesh")) {
        Matlab_Mesh(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver l'acceleration du Centre de masse
    if(!toLower(cmd).compare("patch")) {
        Matlab_Patch(nlhs, plhs, nrhs, prhs);
        return;
    }

    // moment angulaire du centre de masse
    if(!toLower(cmd).compare("comangularmomentum")) {
        Matlab_CoMangularMomentum(nlhs, plhs, nrhs, prhs);
        return;
    }

    // moment angulaire du centre de masse
    if(!toLower(cmd).compare("segmentangularmomentum")) {
        Matlab_segmentAngularMomentum(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver le Centre de masse des segments
    if(!toLower(cmd).compare("segmentcom")) {
        Matlab_segmentCOM(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver la vitesse Centre de masse des segments
    if(!toLower(cmd).compare("segmentcomdot")) {
        Matlab_segmentCOMdot(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction pour trouver l'accélération Centre de masse des segments
    if(!toLower(cmd).compare("segmentcomddot")) {
        Matlab_segmentCOMddot(nlhs, plhs, nrhs, prhs);
        return;
    }


    if(!toLower(cmd).compare("segmentsvelocities")) {
        Matlab_segmentsVelocities(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Inerties segmentaires
    if(!toLower(cmd).compare("segmentsinertia")) {
        Matlab_segmentsInertia(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("segmentsinertialocal")) {
        Matlab_segmentsInertiaLocal(nlhs, plhs, nrhs, prhs);
        return;
    }


    // Obtenir les JCS dans le global
    if(!toLower(cmd).compare("globaljcs")) {
        Matlab_globalJCS(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Obtenir les JCS dans le global
    if(!toLower(cmd).compare("localjcs")) {
        Matlab_localJCS(nlhs, plhs, nrhs, prhs);
        return;
    }

#ifdef MODULE_MUSCLES
    // Obtenir le nombre de muscles
    if(!toLower(cmd).compare("nmuscles")) {
        Matlab_nMuscles(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Obtenir le noms des muscles
    if(!toLower(cmd).compare("musclesnames")) {
        Matlab_MusclesNames(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Obtenir le noms des muscles
    if(!toLower(cmd).compare("musclesparentnames")) {
        Matlab_MusclesParentNames(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Obtenir la position des muscles dans le global
    if(!toLower(cmd).compare("musclepoints")) {
        Matlab_MusclesPoints(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("muscleupdate")) {
        Matlab_muscleUpdate(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Obtenir la position des muscles dans le global
    if(!toLower(cmd).compare("musclelength")) {
        Matlab_MusclesLength(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Obtenir la position des muscles dans le global
    if(!toLower(cmd).compare("muscletendonlength")) {
        Matlab_MusclesTendonLength(nlhs, plhs, nrhs, prhs);
        return;
    }


    // Obtenir la vitesse d'élongation des muscles
    if(!toLower(cmd).compare("musclevelocity")) {
        Matlab_muscleVelocity(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de dynamique inverse
    if(!toLower(cmd).compare("inversedynamics")) {
        Matlab_inverseDynamics(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("jointtorquefromforce")) {
        Matlab_muscleJointTorqueFromMuscleForce(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("jointtorquefromactivation")) {
        Matlab_muscleJointTorqueFromActivation(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("jointtorquefromexcitation")) {
        Matlab_muscleJointTorqueFromExcitation(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("muscleactivationdot")) {
        Matlab_MusclesActivationDot(nlhs, plhs, nrhs, prhs);
        return;
    }
    if(!toLower(cmd).compare("muscleexcitationdotbuchanan")) {
        Matlab_MusclesExcitationDotBuchanan(nlhs, plhs, nrhs, prhs);
        return;
    }


    if(!toLower(cmd).compare("muscleforce")
            || !toLower(cmd).compare("muscleforces")
            || !toLower(cmd).compare("muscleforcenorm")) {
        Matlab_MusclesForce(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("muscleforcemax")) {
        Matlab_MusclesForceMax(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("musclejacobian")) {
        Matlab_MusclesJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("musclelengthjacobian")) {
        Matlab_muscleLengthJacobian(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("changeshapefactors")) {
        Matlab_ChangeShapeFactors(nlhs, plhs, nrhs, prhs);
        return;
    }

#endif // MODULE_MUSCLES

#ifdef MODULE_ACTUATORS
    // Si on veut convertir des activations de torque en torque
    if(!toLower(cmd).compare("torqueactivation")) {
        Matlab_torqueActivation(nlhs, plhs, nrhs, prhs);
        return;
    }
#endif // MODULE_ACTUATORS


    if(!toLower(cmd).compare("forwarddynamics")) {
        Matlab_forwardDynamics(nlhs, plhs, nrhs, prhs);
        return;
    }

    if(!toLower(cmd).compare("computeqdot")) {
        Matlab_computeQdot(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Fonction de dynamique inverse
    if(!toLower(cmd).compare("nleffects")) {
        Matlab_NLeffects(nlhs, plhs, nrhs, prhs);
        return;
    }

    // Got here, so command is not recognized
    mexErrMsgTxt("Command for interface not recognized.");
}

#endif // MATLAB_BIORBD_H
