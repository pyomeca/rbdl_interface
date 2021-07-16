#ifndef BIORBD_MATLAB_CHANGE_GRAVITY_H
#define BIORBD_MATLAB_CHANGE_GRAVITY_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_changeGravity   ( int, mxArray *plhs[],
                              int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3,
                               "3 arguments are required where the 2nd is the handler on the model and 3rd is the gravity field 3d vector");
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( 1, 1, mxREAL);
    model->gravity = getVector3d(prhs, 2);
}

#endif // BIORBD_MATLAB_CHANGE_GRAVITY_H
