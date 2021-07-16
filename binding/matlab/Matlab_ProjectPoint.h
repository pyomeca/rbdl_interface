#ifndef BIORBD_MATLAB_PROJECT_POINT_H
#define BIORBD_MATLAB_PROJECT_POINT_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_ProjectPoint( int, mxArray *plhs[],
                          int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 4, 4,
                               "6 arguments are required where the 2nd is the handler on the model, 3rd is the Q, 4th are the 3xNxT points in global reference frame where N = nMarkers of the model");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF

    // Recevoir Q
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> Qall = getParameterQ(
                prhs, 2, nQ);

    // Récupérer les marqueurs selon que l'on veut tous ou seulement anatomiques ou techniques
    std::vector<std::vector<biorbd::rigidbody::NodeSegment>> markersOverTime =
                getParameterAllMarkers(prhs,3);

    unsigned int nFrames(static_cast<unsigned int>(markersOverTime.size()));
    if (Qall.size()!=nFrames) {
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension",
                           "Q must have the same number of frames than markers");
    }
    int nMarker(static_cast<int>((*(markersOverTime.begin())).size()));

    // Create a matrix for the return argument
    mwSize dims[3];
    dims[0] = 3;
    dims[1] = static_cast<mwSize>(nMarker);
    dims[2] = nFrames;
    plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
    double *Markers = mxGetPr(plhs[0]);

    // Projeter les points
    unsigned int cmp(0);
    for (unsigned int i=0; i<nFrames; ++i) {
        biorbd::rigidbody::GeneralizedCoordinates Q(*(Qall.begin()+i));
        std::vector<biorbd::rigidbody::NodeSegment> projectedPoint(model->projectPoint(
                    Q, *(markersOverTime.begin()+i), true));
        for (unsigned int j=0; j<static_cast<unsigned int>(nMarker); ++j) {
            biorbd::rigidbody::NodeSegment tp(*(projectedPoint.begin()+j));
            Markers[cmp+0] = tp(0);
            Markers[cmp+1] = tp(1);
            Markers[cmp+2] = tp(2);
            cmp += 3;
        }
    }

    return;
}

#endif // BIORBD_MATLAB_PROJECT_POINT_H
