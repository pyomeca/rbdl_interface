#ifndef BIORBD_MATLAB_IMU_JACOBIAN_H
#define BIORBD_MATLAB_IMU_JACOBIAN_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_IMUJacobian( int, mxArray *plhs[],
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


    // Trouver la matrice jacobienne de tous les marqueurs
    std::vector<biorbd::utils::Matrix> Jac_tp = model->IMUJacobian(Q);
    std::vector<biorbd::utils::Matrix>::iterator it=Jac_tp.begin();

    // Create a matrix for the return argument
    unsigned int nIMUs = model->nbIMUs();
    plhs[0] = mxCreateDoubleMatrix( 9*nIMUs, nQ, mxREAL);
    double *Jac = mxGetPr(plhs[0]);
    int cmp(0);
    for (unsigned int i=0; i<nQ; ++i)
        for (unsigned int j=0; (it+j)!=Jac_tp.end(); ++j)
            for (unsigned int k=0; k<9; ++k) {
                Jac[cmp] = (*(it+j))(k,i);
                ++cmp;
            }

    return;
}

#endif // BIORBD_MATLAB_IMU_JACOBIAN_H
