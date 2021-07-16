#ifndef BIORBD_MATLAB_TORQUE_ACTIVATION_H
#define BIORBD_MATLAB_TORQUE_ACTIVATION_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_torqueActivation( int, mxArray *plhs[],
                              int nrhs, const mxArray*prhs[] )
{
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 5, 5,
                               "5 or 5 arguments are required where the 2nd is the handler on the model, 3rd is the Q, 4th is QDot and 5th is GeneralizedTorque");

    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF
    unsigned int nQdot = model->nbQdot(); // Get the number of DoF
    unsigned int nGeneralizedTorque =
        model->nbGeneralizedTorque(); // Nombre de GeneralizedTorque

    // Recevoir Q
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> Q = getParameterQ(prhs,
            2, nQ);
    // Recevoir Qdot
    std::vector<biorbd::rigidbody::GeneralizedVelocity> QDot = getParameterQdot(
                prhs, 3, nQdot);
    // Recevoir Qddot
    std::vector<biorbd::rigidbody::GeneralizedTorque> act =
        getParameterGeneralizedTorque(prhs, 4,
                                      model->nbGeneralizedTorque());

    // S'assurer que Q, Qdot et Qddot (et Forces s'il y a lieu) sont de la bonne dimension
    unsigned int nFrame(static_cast<unsigned int>(Q.size()));
    if (QDot.size() != nFrame) {
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension",
                           "QDot must have the same number of frames than Q");
    }
    if (act.size() != nFrame) {
        mexErrMsgIdAndTxt( "MATLAB:dim:WrongDimension",
                           "GeneralizedTorque must have the same number of frames than Q");
    }

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix(model->nbGeneralizedTorque() , nFrame, mxREAL);
    double *GeneralizedTorque = mxGetPr(plhs[0]);
    unsigned int cmp(0);

    // Trouver la dynamique inverse a cette configuration
    for (unsigned int j=0; j<Q.size(); ++j) {
        // Calcul des couples
        Eigen::VectorXd Tau(model->torque(act[j], Q[j], QDot[j]));

        // Remplir l'output
        for (unsigned int i=0; i<nGeneralizedTorque; i++) {
            GeneralizedTorque[cmp] = Tau(i);
            ++cmp;
        }
    }

    return;
}

#endif // BIORBD_MATLAB_TORQUE_ACTIVATION_H
