#ifndef BIORBD_MATLAB_N_Q_H
#define BIORBD_MATLAB_N_Q_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_nQ(int, mxArray *plhs[],
               int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 2, 2,
                               "2 arguments are required where the 2nd is the handler on the model");
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( 1, 1, mxREAL);
    double *nQ = mxGetPr(plhs[0]);

    // Get nombre de degrés de liberté
    *nQ = model->nbQ();

    return;
}

#endif // BIORBD_MATLAB_N_Q_H
