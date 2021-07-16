#ifndef BIORBD_MATLAB_INVERSE_KINEMATICS_H
#define BIORBD_MATLAB_INVERSE_KINEMATICS_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_inverseKinematics( int, mxArray *plhs[],
                               int nrhs, const mxArray*prhs[] )
{
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 4, 5,
                               "4 arguments are required [+1 optional] where the 2nd is the handler on the model, 3rd is the 3xNxTime marker positions matrix and 4th is the initial guess for Q and 5th if you want to remove axes as specified in the model file [default = true]");

    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF

    // Recevoir la matrice des markers
    std::vector<std::vector<biorbd::rigidbody::NodeSegment>> markersOverTime =
                getParameterAllMarkers(prhs,2,static_cast<int>(model->nbTechnicalMarkers()));

    // Recevoir Qinit
    biorbd::rigidbody::GeneralizedCoordinates Qinit = *getParameterQ(prhs, 3,
            nQ).begin();

    bool removeAxes(true);
    if (nrhs >= 5) {
        removeAxes = getBool(prhs, 4);
    }

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix(nQ , markersOverTime.size(), mxREAL);
    double *q = mxGetPr(plhs[0]);

    // Faire la cinématique inverse a chaque instant
    for (unsigned int i=0; i<markersOverTime.size(); ++i) {
        biorbd::rigidbody::GeneralizedCoordinates Q(nQ);
        Q.setZero();

        // Faire la cinématique inverse
        model->inverseKinematics(*(markersOverTime.begin()+i), Qinit, Q);

        // Remplir la variable de sortie
        for (unsigned int j=0; j<nQ; ++j) {
            q[i*nQ+j] = Q[j];
        }

        // Q devient Qinit
        Qinit = Q;
    }

    return;
}

#endif // BIORBD_MATLAB_INVERSE_KINEMATICS_H
