#ifndef BIORBD_MATLAB_SEGMENT_COM_DOT_H
#define BIORBD_MATLAB_SEGMENT_COM_DOT_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_segmentCOMdot( int, mxArray *plhs[],
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
    biorbd::rigidbody::GeneralizedCoordinates Q = *getParameterQ(prhs, 2,
            nQ).begin();
    // Recevoir Qdot
    biorbd::rigidbody::GeneralizedVelocity QDot = *getParameterQdot(prhs, 3,
            nQdot).begin();
    // Recevoir le numéro du segment (optionnel)
    int i(0);
    if (nrhs==5) {
        i = getInteger(prhs, 4);
    }
    i -= 1; // -1 car le segment 0 est numéroté 1 sous matlab

    // Trouver la vitesse du CoM
    if (i==-1) {
        std::vector<biorbd::utils::Vector3d> COMdot = model->CoMdotBySegment(Q,QDot,
                true);
        // Create a matrix for the return argument
        plhs[0] = mxCreateDoubleMatrix( 3, model->nbSegment(), mxREAL);
        // Remplir l'output
        double *tp = mxGetPr(plhs[0]);
        for (unsigned int j=0; j<model->nbSegment(); ++j)
            for (unsigned int k=0; k<3; ++k) {
                tp[3*j+k] = COMdot[j][k];    // Transférer le tout dans un tableau de sortie
            }
    } else {
        biorbd::utils::Vector3d COMdot = model->CoMdotBySegment(Q,QDot,
                                         static_cast<unsigned int>(i),true);

        // Create a matrix for the return argument
        plhs[0] = mxCreateDoubleMatrix( 3, 1, mxREAL);
        // Remplir l'output
        double *tp = mxGetPr(plhs[0]);
        for (unsigned int k=0; k<3; ++k) {
            tp[k] = COMdot[k];    // Transférer le tout dans un tableau de sortie
        }
    }

    return;
}

#endif // BIORBD_MATLAB_SEGMENT_COM_DOT_H
