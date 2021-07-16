#ifndef BIORBD_MATLAB_N_MUSCLES_H
#define BIORBD_MATLAB_N_MUSCLES_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_nMuscles( int, mxArray *plhs[],
                      int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2,
                               "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Sortie des noms
    plhs[0] =  mxCreateDoubleMatrix( 1, 1, mxREAL);
    double *nMus = mxGetPr(plhs[0]);

    // Récupérer le nombre de muscles
    *nMus = model->nbMuscleTotal();

    return;
}

#endif // BIORBD_MATLAB_N_MUSCLES_H
