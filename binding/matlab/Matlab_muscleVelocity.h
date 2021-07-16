#ifndef MATLAB_BIORBD_MUSCLES_VELOCITY_H
#define MATLAB_BIORBD_MUSCLES_VELOCITY_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"
#include "Muscles/Muscle.h"
#include "Muscles/MuscleGroup.h"

void Matlab_muscleVelocity( int, mxArray *plhs[],
                            int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 4, 4,
                               "4 arguments are required where the 2nd is the handler on the model, 3rd is the Q and 4th is the Qdot");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF
    unsigned int nQdot = model->nbQdot(); // Get the number of DoF

    // Recevoir Q
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> Q = getParameterQ(prhs,
            2, nQ);

    // Recevoir Qdot
    std::vector<biorbd::rigidbody::GeneralizedVelocity> Qdot = getParameterQdot(
                prhs, 3, nQdot);

    // S'assurer que le même nombre d'instants a été envoyé
    if (Q.size() != Qdot.size()) {
        mexErrMsgIdAndTxt( "MATLAB:findnz:time",
                           "Time stamps for Q and Qdot don't correspond");
    }

    // Cellules de sortie
    mwSize dims[2];
    dims[0] = model->nbMuscleTotal();
    dims[1] = Q.size();
    plhs[0] = mxCreateNumericArray(2, dims, mxDOUBLE_CLASS, mxREAL);
    double *length = mxGetPr(plhs[0]);

    // Aller chercher les valeurs
    unsigned int cmp(0);
    for (unsigned int iQ=0; iQ<Q.size(); ++iQ)
        for (unsigned int i=0; i<model->nbMuscleGroups(); ++i)
            for (unsigned int j=0; j<model->muscleGroup(i).nbMuscles(); ++j) {
                // Recueillir toutes les longueurs musculaire
                length[cmp]   = model->muscleGroup(i).muscle(j).velocity(*model,Q[iQ],Qdot[iQ],
                                true);
                ++cmp;
            }

    return;
}

#endif // MATLAB_BIORBD_MUSCLES_VELOCITY_H
