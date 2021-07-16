#ifndef BIORBD_MATLAB_MASS_MATRIX_H
#define BIORBD_MATLAB_MASS_MATRIX_H

#include <mex.h>
#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Dynamics.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_massMatrix( int, mxArray *plhs[],
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

    // Create a matrix for the return argument
    mwSize ndim(3);
    mwSize dim[3] = {nQ, nQ, Q.size()};
    plhs[0] = mxCreateNumericArray(ndim, dim, mxDOUBLE_CLASS, mxREAL);
    double *mass = mxGetPr(plhs[0]);

    unsigned int cmp(0);
    // Trouver la matrice de masse
    RigidBodyDynamics::Math::MatrixNd Mass(nQ, nQ);
    for (unsigned int j = 0; j<Q.size(); ++j) {
        Mass.setZero();
        model->UpdateKinematicsCustom(&Q[j], nullptr, nullptr);
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(*model, Q[j], Mass, false);

        // Remplir l'output
        for (unsigned int i=0; i<nQ*nQ; ++i) {
            mass[cmp + i] = Mass(i);
        }
        cmp += nQ*nQ;
    }

    return;
}

#endif // BIORBD_MATLAB_MASS_MATRIX_H
