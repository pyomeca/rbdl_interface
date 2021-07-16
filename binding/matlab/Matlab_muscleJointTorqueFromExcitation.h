#ifndef MATLAB_BIORBD_MUSCLES_JOINT_TORQUE_FROM_EXCITATION_H
#define MATLAB_BIORBD_MUSCLES_JOINT_TORQUE_FROM_EXCITATION_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"
#include "Muscles/MuscleGroup.h"
#include "Muscles/Muscle.h"

void Matlab_muscleJointTorqueFromExcitation( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 5, 6,
                               "5 arguments are required where the 2nd is the handler on the model, "
                               "3rd is the Q, 4th is QDot, 5th is the muscles excitation states and optional "
                               "6th is if update [true] must be done. Note that if update is set to [false], "
                               "the user MUST update it by himself before calling this function");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF
    unsigned int nQdot = model->nbQdot(); // Get the number of DoF
    unsigned int nGeneralizedTorque =
        model->nbGeneralizedTorque(); // Get the number of DoF
    unsigned int nMuscleTotal = model->nbMuscleTotal();

    // Recevoir Q
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> Q = getParameterQ(prhs,
            2, nQ);
    // Recevoir Qdot
    std::vector<biorbd::rigidbody::GeneralizedVelocity> QDot = getParameterQdot(
                prhs, 3, nQdot);
    // Recevoir muscleStates
    std::vector<std::vector<std::shared_ptr<biorbd::muscles::State>>> s =
        getParameterMuscleStateExcitation(prhs,4,
                                          model->nbMuscleTotal());

    // S'assurer que Q, Qdot et Qddot (et Forces s'il y a lieu) sont de la bonne dimension
    unsigned int nFrame(static_cast<unsigned int>(Q.size()));
    if (QDot.size() != nFrame) {
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension",
                           "QDot must have the same number of frames than Q");
    }
    if (s.size() != nFrame) {
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension",
                           "Muscles states must have the same number of frames than Q");
    }

    bool updateKin(true);
    if (nrhs >= 6) { // Si on doit récupérer si on update ou pas
        updateKin = getBool(prhs, 5);
        if (!updateKin
                && nFrame > 1) { // Si on n'update pas, s'assurer qu'un seul frame est envoyé
            mexErrMsgIdAndTxt("MATLAB:dim:WrongDimension",
                              "No update is incompatible with more than one frame.");
        }
    }

    // Create a matrix for the return argument
    mwSize dims[2];
    dims[0] = nGeneralizedTorque;
    dims[1] = nFrame;
    plhs[0] = mxCreateNumericArray(2, dims, mxDOUBLE_CLASS, mxREAL);
    double *GeneralizedTorque = mxGetPr(plhs[0]);

    // Si la personne a demandé de sortir les forces par muscle également
    double *Mus = nullptr;
    if (nlhs >= 2) {
        mwSize dims2[2];
        dims2[0] = nMuscleTotal;
        dims2[1] = nFrame;
        plhs[1] = mxCreateNumericArray(2, dims2, mxDOUBLE_CLASS, mxREAL);
        Mus = mxGetPr(plhs[1]);
    }

    // Remplir le output
    biorbd::rigidbody::GeneralizedTorque muscleTorque;
    biorbd::utils::Vector muscleForces;
    for (unsigned int i=0; i<nFrame; ++i) {
        unsigned int iMus = 0;
        for (unsigned int k=0; k<model->nbMuscleGroups(); ++k)
            for (unsigned int j=0; j<model->muscleGroup(k).nbMuscles(); ++j) {
                std::dynamic_pointer_cast<biorbd::muscles::StateDynamics>(s[i][iMus])
                ->timeDerivativeActivation(model->muscleGroup(k).muscle(j).characteristics(),
                                           true);
                ++iMus;
            }

        if (updateKin) {
            muscleForces = model->muscleForces(s[i], Q[i], QDot[i]);
        } else {
            muscleForces = model->muscleForces(s[i]);
        }
        muscleTorque = model->muscularJointTorque(muscleForces);

        // distribuer les GeneralizedTorque
        for (unsigned int j=0; j<nGeneralizedTorque; ++j) {
            GeneralizedTorque[i*nGeneralizedTorque+j] = muscleTorque(j);
        }

        // Distribuer les forces
        if (nlhs >= 2) // Si on doit récupérer les forces
            for (unsigned int j=0; j<nMuscleTotal; ++j) {
                Mus[i*nMuscleTotal+j] = muscleForces(j);
            }
    }

    return;
}

#endif // MATLAB_BIORBD_MUSCLES_JOINT_TORQUE_FROM_EXCITATION_H
