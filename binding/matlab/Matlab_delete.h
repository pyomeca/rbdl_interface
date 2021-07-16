#ifndef BIORBD_MATLAB_DELETE_H
#define BIORBD_MATLAB_DELETE_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_delete( int nlhs, mxArray *[],
                    int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 2, 2,
                               "2 arguments are required where the 2nd is the handler on the model");

    // Destroy the C++ object
    destroyObject<biorbd::Model>(prhs[1]);
    // Warn if other commands were ignored
    if (nlhs != 0 || nrhs != 2) {
        mexWarnMsgTxt("Delete: Unexpected output arguments ignored.");
    }
    return;
}

#endif // BIORBD_MATLAB_DELETE_H¸
