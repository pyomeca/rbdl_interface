#ifndef BIORBD_MATLAB_PROJECT_POINT_JACOBIAN_H
#define BIORBD_MATLAB_PROJECT_POINT_JACOBIAN_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_ProjectPointJacobian( int, mxArray *plhs[],
                                  int nrhs, const mxArray*prhs[] )
{
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 4, 4,
                               "3 arguments are required where the 2nd is the handler on the model and 3rd is the Q and 4th are the 3xN markers where N=nMarkers of the model");

    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF

    // Recevoir Q
    biorbd::rigidbody::GeneralizedCoordinates Q = *getParameterQ(prhs, 2,
            nQ).begin();

    // Récupérer les marqueurs selon que l'on veut tous ou seulement anatomiques ou techniques
    std::vector<biorbd::rigidbody::NodeSegment> markersOverTime =
        *getParameterAllMarkers(prhs,3).begin();

    // Trouver la matrice jacobienne de tous les marqueurs
    std::vector<biorbd::utils::Matrix> Jac_tp = model->projectPointJacobian(Q,
            markersOverTime, true);
    std::vector<biorbd::utils::Matrix>::iterator it=Jac_tp.begin();

    // Create a matrix for the return argument
    unsigned int nMarkers = model->nbMarkers();
    plhs[0] = mxCreateDoubleMatrix( 3*nMarkers, nQ, mxREAL);
    double *Jac = mxGetPr(plhs[0]);
    for (unsigned int j=0; (it+j)!=Jac_tp.end(); ++j)
        for (unsigned int i=0; i<nQ; ++i)
            for (unsigned int k=0; k<3; ++k) {
                Jac[j+nMarkers*k+i*nMarkers*3] = (*(it+j))(k,i);
            }

    return;
}

#endif // BIORBD_MATLAB_PROJECT_POINT_JACOBIAN_H
