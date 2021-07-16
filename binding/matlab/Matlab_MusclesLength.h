#ifndef BIORBD_MATLAB_MUSCLES_LENGTH_H
#define BIORBD_MATLAB_MUSCLES_LENGTH_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"
#include "Muscles/Muscle.h"
#include "Muscles/MuscleGroup.h"

void Matlab_MusclesLength( int, mxArray *plhs[],
                           int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3,
                               "3 arguments are required where the 2nd is the handler on the model and 3rd is the Q");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF

    // Recevoir Q
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> Q = getParameterQ(prhs,
            2, nQ);

    // Cellules de sortie
    mwSize dims[2];
    dims[0] = model->nbMuscleTotal();
    dims[1] = Q.size();
    plhs[0] = mxCreateNumericArray(2, dims, mxDOUBLE_CLASS, mxREAL);
    double *length = mxGetPr(plhs[0]);

    // Aller chercher les valeurs
    unsigned int cmp(0);
    for (unsigned int iQ=0; iQ<Q.size(); ++iQ) {
        int updateKin(2);
        for (unsigned int i=0; i<model->nbMuscleGroups(); ++i)
            for (unsigned int j=0; j<model->muscleGroup(i).nbMuscles(); ++j) {
                // Recueillir toutes les longueurs musculaire
                length[cmp]   = model->muscleGroup(i).muscle(j).length(*model,Q[iQ],updateKin);
                updateKin = 1;
                ++cmp;
            }
    }

    return;
}

void Matlab_MusclesTendonLength( int, mxArray *plhs[],
                                 int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3,
                               "3 arguments are required where the 2nd is the handler on the model and 3rd is the Q");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF

    // Recevoir Q
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> Q = getParameterQ(prhs,
            2, nQ);

    // Cellules de sortie
    mwSize dims[2];
    dims[0] = model->nbMuscleTotal();
    dims[1] = Q.size();
    plhs[0] = mxCreateNumericArray(2, dims, mxDOUBLE_CLASS, mxREAL);
    double *length = mxGetPr(plhs[0]);

    // Aller chercher les valeurs
    unsigned int cmp(0);
    for (unsigned int iQ=0; iQ<Q.size(); ++iQ) {
        int updateKin(2);
        for (unsigned int i=0; i<model->nbMuscleGroups(); ++i)
            for (unsigned int j=0; j<model->muscleGroup(i).nbMuscles(); ++j) {
                // Recueillir toutes les longueurs musculaire
                length[cmp]   = model->muscleGroup(i).muscle(j).musculoTendonLength(*model,
                                Q[iQ],updateKin);
                updateKin = 1;
                ++cmp;
            }
    }

    return;
}

#endif // BIORBD_MATLAB_MUSCLES_LENGTH_H
