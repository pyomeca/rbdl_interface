#ifndef BIORBD_MATLAB_SEGMENT_ANGULAR_MOMENTUM_H
#define BIORBD_MATLAB_SEGMENT_ANGULAR_MOMENTUM_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_segmentAngularMomentum( int, mxArray *plhs[],
                                    int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 4, 5,
                               "4 arguments are required (+1 optional) where the 2nd is the handler on the model, 3rd is the Q, 4th is the Qdot and 5th is the index of body segment");

    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF
    unsigned int nQdot = model->nbQdot(); // Get the number of DoF

    // Recevoir Q
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> Q = getParameterQ(prhs,
            2, nQ);
    // Recevoir QDot
    std::vector<biorbd::rigidbody::GeneralizedVelocity> QDot = getParameterQdot(
                prhs, 3, nQdot);

    unsigned int nFrame(static_cast<unsigned int>(Q.size()));
    if (QDot.size() != nFrame) {
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension",
                           "QDot must have the same number of frames than Q");
    }

    // Recevoir l'index du segment
    int idx(-1);
    if (nrhs==5) {
        idx = getInteger(prhs, 4) - 1; // -1 ainsi le premier segment est 1
        if (idx >= static_cast<int>(nQ) || idx < 0)
            mexErrMsgIdAndTxt("MATLAB:model:invalidSegmentInput",
                              "Selected segment doesn't exist");
    }

    // Create a matrix for the return argument
    if (idx == -1) {
        mwSize dims[3];
        dims[0] = 3;
        dims[1] = nQ;
        dims[2] = nFrame;
        plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
    } else {
        mwSize dims[3];
        dims[0] = 3;
        dims[1] = 1;
        dims[2] = nFrame;
        plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
    }
    double *angularMomentum = mxGetPr(plhs[0]);


    // Préparer la sortie conditionnelle (tous si idx==-1, seulement celui demandé sinon)
    unsigned int cmp(0);
    for (unsigned int i=0; i<nFrame; ++i) {
        std::vector<biorbd::utils::Vector3d> am_all(model->CalcSegmentsAngularMomentum (
                    *model, *(Q.begin()+i),
                    *(QDot.begin()+i), true));
        if (idx==-1) {
            // Remplir le output
            for (auto am : am_all) {
                angularMomentum[cmp*3  ] = am(0);
                angularMomentum[cmp*3+1] = am(1);
                angularMomentum[cmp*3+2] = am(2);
                cmp++;
            }
        } else {
            // Remplir le output
            angularMomentum[i*3+0] = am_all[0](0);
            angularMomentum[i*3+1] = am_all[0](1);
            angularMomentum[i*3+2] = am_all[0](2);
        }
    }

    return;
}

#endif // BIORBD_MATLAB_SEGMENT_ANGULAR_MOMENTUM_H
