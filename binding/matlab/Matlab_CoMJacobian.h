#ifndef BIORBD_MATLAB_COM_JACOBIAN_H
#define BIORBD_MATLAB_COM_JACOBIAN_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"
#include "Utils/Matrix.h"

void Matlab_CoMJacobian( int, mxArray *plhs[],
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

    // Trouver la jacobienne du COM
    RigidBodyDynamics::Math::MatrixNd Jaco(model->CoMJacobian(Q).transpose());

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( 3, nQ, mxREAL);
    double *jaco = mxGetPr(plhs[0]);

    // Mettre les valeurs dans la sortie;
    for (unsigned int i=0; i<nQ*3; ++i) {
        jaco[i] = Jaco(i);
    }

    return;
}

#endif // BIORBD_MATLAB_COM_JACOBIAN_H
