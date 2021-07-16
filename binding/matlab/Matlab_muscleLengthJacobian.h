#ifndef MATLAB_BIORBD_MUSCLES_LENGTH_JACOBIAN_H
#define MATLAB_BIORBD_MUSCLES_LENGTH_JACOBIAN_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_muscleLengthJacobian( int, mxArray *plhs[],
                                  int nrhs, const mxArray*prhs[] )
{
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3,
                               "3 arguments are required where the 2nd is the handler on the model and 3rd is the Q");

    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF

    // Recevoir Q
    biorbd::rigidbody::GeneralizedCoordinates Q = *getParameterQ(prhs, 2,
            nQ).begin();

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( model->nbMuscleTotal(), nQ, mxREAL);
    double *Jac = mxGetPr(plhs[0]);

    biorbd::utils::Matrix jaco(model->musclesLengthJacobian(Q));
    int cmp(0);
    for (unsigned int j=0; j<jaco.cols(); ++j)
        for (unsigned int i=0; i<jaco.rows(); ++i) {
            Jac[cmp] = jaco(i,j);
            ++cmp;
        }

    return;
}

#endif // MATLAB_BIORBD_MUSCLES_LENGTH_JACOBIAN_H
