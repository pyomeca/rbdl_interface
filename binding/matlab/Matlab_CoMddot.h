#ifndef BIORBD_MATLAB_COM_DDOT_H
#define BIORBD_MATLAB_COM_DDOT_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_CoMddot( int nlhs, mxArray *plhs[],
                     int nrhs, const mxArray*prhs[] )
{
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 5, 5,
                               "4 arguments are required where the 2nd is the handler on the model, 3rd is the Q and 4th is QDot");

    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF
    unsigned int nQdot = model->nbQdot(); // Get the number of DoF
    unsigned int nQddot = model->nbQddot(); // Get the number of DoF

    // Recevoir Q
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> Q = getParameterQ(prhs,
            2, nQ);
    // Recevoir Qdot
    std::vector<biorbd::rigidbody::GeneralizedVelocity> QDot = getParameterQdot(
                prhs, 3, nQdot);
    // Recevoir Qddot
    std::vector<biorbd::rigidbody::GeneralizedAcceleration> QDDot =
        getParameterQddot(prhs, 4, nQddot);

    // S'assurer que Q, Qdot et Qddot (et Forces s'il y a lieu) sont de la bonne dimension
    unsigned int nFrame(Q.size());
    if (QDot.size() != nFrame) {
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension",
                           "QDot must have the same number of frames than Q");
    }
    if (QDDot.size() != nFrame) {
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension",
                           "QDDot must have the same number of frames than Q");
    }

    // Create a matrix for the return argument
    mwSize dims[3];
    dims[0] = 3;
    dims[1] = 1;
    dims[2] = Q.size();
    plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
    double *com_ddot = mxGetPr(plhs[0]);

    // Trouver l'accélération du CoM
    for (unsigned int i=0; i<Q.size(); ++i) {
        RigidBodyDynamics::Math::Vector3d COMDDot = model->CoMddot(Q[i], QDot[i],
                QDDot[i]);
        for (unsigned int j=0; j<3; ++j) {
            com_ddot[3*i+j] = COMDDot(j);
        }
    }

    return;
}

#endif // BIORBD_MATLAB_COM_DDOT_H
