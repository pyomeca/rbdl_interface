#ifndef BIORBD_MATLAB_SEGMENT_VELOCITIES_H
#define BIORBD_MATLAB_SEGMENT_VELOCITIES_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"
#include "Utils/Vector.h"

void Matlab_segmentsVelocities( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] )
{
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 4, 4,
                               "4 arguments are required where the 2nd is the handler on the model, 3rd is the Q and 4th is QDot");

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

    // Update sur la cinématique
    model->UpdateKinematicsCustom(&Q, &QDot, nullptr);

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( 6, nQ+1, mxREAL);
    double *vel = mxGetPr(plhs[0]);

    // Remplir l'output
    unsigned int cmp = 0;
    for (std::vector<RigidBodyDynamics::Math::SpatialVector>::iterator v_it =
                model->v.begin(); v_it != model->v.end();
            ++v_it) {
        for (unsigned int i = 0; i<6; ++i) {
            vel[i+6*cmp] = (*v_it)(i);
        }
        ++cmp;
    }

    return;
}

#endif // BIORBD_MATLAB_SEGMENT_VELOCITIES_H
