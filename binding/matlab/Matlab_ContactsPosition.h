#ifndef BIORBD_MATLAB_CONTACS_POSITION_H
#define BIORBD_MATLAB_CONTACS_POSITION_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_ContactsPosition( int, mxArray *plhs[],
                              int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3,
                               "3 arguments are required where the 2nd is the handler on the model and 3rd is the Q");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF

    // Recevoir Q
    biorbd::rigidbody::GeneralizedCoordinates Q = *getParameterQ(prhs, 2,
            nQ).begin();

    // Trouver où sont les marqueurs
    std::vector<biorbd::utils::Vector3d> Contact_tp = model->constraintsInGlobal(Q,
            true);


    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( 3, Contact_tp.size(), mxREAL);
    double *contact = mxGetPr(plhs[0]);

    // Remplir le output
    std::vector<biorbd::utils::Vector3d>::iterator it=Contact_tp.begin();
    for (unsigned int i=0; (it+i)!=Contact_tp.end(); ++i) {
        contact[i*3] = (*(it+i))(0);
        contact[i*3+1] = (*(it+i))(1);
        contact[i*3+2] = (*(it+i))(2);
    }

    return;
}

#endif // BIORBD_MATLAB_CONTACS_POSITION_H
