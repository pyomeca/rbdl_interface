#ifndef BIORBD_MATLAB_NL_EFFECTS_H
#define BIORBD_MATLAB_NL_EFFECTS_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_NLeffects( int, mxArray *plhs[],
                       int nrhs, const mxArray*prhs[] )
{
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 4, 4,
                               "4 arguments are required where the 2nd is the handler on the model, 3rd is the Q and 4th is QDot");

    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF
    unsigned int nQdot = model->nbQdot(); // Get the number of DoF
    unsigned int nTau = model->nbGeneralizedTorque(); // Nombre de GeneralizedTorque

    // Recevoir Q
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> Q = getParameterQ(prhs,
            2, nQ);
    // Recevoir Qdot
    std::vector<biorbd::rigidbody::GeneralizedVelocity> QDot = getParameterQdot(
                prhs, 3, nQdot);

    // S'assurer que Q, Qdot et Qddot (et Forces s'il y a lieu) sont de la bonne dimension
    unsigned int nFrame(static_cast<unsigned int>(Q.size()));
    if (QDot.size() != nFrame) {
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension",
                           "QDot must have the same number of frames than Q");
    }

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix(nTau , nFrame, mxREAL);
    double *tau = mxGetPr(plhs[0]);
    unsigned int cmp(0);

    // Trouver les effets non-linéaires pour chaque configuration
    biorbd::rigidbody::GeneralizedTorque Tau(nTau);
    for (unsigned int j=0; j<Q.size(); ++j) {
        Tau.setZero();
        RigidBodyDynamics::NonlinearEffects(*model, Q[j], QDot[j], Tau);

        // Remplir l'output
        for (unsigned int i=0; i<nTau; i++) {
            tau[cmp] = Tau(i);
            ++cmp;
        }
    }

    return;
}

#endif // BIORBD_MATLAB_NL_EFFECTS_H
