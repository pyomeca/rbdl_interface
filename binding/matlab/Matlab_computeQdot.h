#ifndef BIORBD_MATLAB_COMPUTE_Q_DOT_H
#define BIORBD_MATLAB_COMPUTE_Q_DOT_H

#include <mex.h>
#include <rbdl/Model.h>
#include <rbdl/Kinematics.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_computeQdot( int, mxArray *plhs[],
                         int nrhs, const mxArray*prhs[] )
{
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 4, 4,
                               "4 are required where the 2nd is the handler on the model, 3rd is the Q and 4th is QDot");

    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ();
    unsigned int nQdot = model->nbQdot();

    // Recevoir Q
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> Q = getParameterQ(prhs,
            2, nQ);
    // Recevoir Qdot
    std::vector<biorbd::rigidbody::GeneralizedVelocity> QDot = getParameterQdot(
                prhs, 3, nQdot);

    // S'assurer que Q et Qdot sont de la bonne dimension
    unsigned int nFrame(static_cast<unsigned int>(Q.size()));

    if (QDot.size() != nFrame) {
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension",
                           "QDot must have the same number of frames than Q");
    }

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( nQ, nFrame, mxREAL);
    double *qdotPost = mxGetPr(plhs[0]);
    unsigned int cmp(0);

    for (unsigned int j=0; j<nFrame; ++j) {
        model->UpdateKinematicsCustom(&Q[j], &QDot[j], nullptr);

        // Trouver la dynamique directe a cette configuration
        biorbd::rigidbody::GeneralizedCoordinates QDotPost(
            model->computeQdot(Q[j], QDot[j]));

        // Remplir l'output
        for (unsigned int i=0; i<nQ; i++) {
            qdotPost[cmp] = QDotPost(i);
            ++cmp;
        }
    }

    return;
}

#endif // BIORBD_MATLAB_COMPUTE_Q_DOT_H
