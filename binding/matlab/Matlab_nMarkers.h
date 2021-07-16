#ifndef BIORBD_MATLAB_N_MARKERS_H
#define BIORBD_MATLAB_N_MARKERS_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_nMarkers( int, mxArray *plhs[],
                      int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 2, 2,
                               "2 arguments are required where the 2nd is the handler on the model");
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( 1, 1, mxREAL);
    double *nMarkers = mxGetPr(plhs[0]);

    // Get nombre de marqueurs
    *nMarkers = static_cast<double>(model->nbMarkers());

    return;
}

#endif // BIORBD_MATLAB_N_MARKERS_H
