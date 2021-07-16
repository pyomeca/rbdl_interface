#ifndef BIORBD_MATLAB_INVERSE_KINEMATICS_EKF_IMU_H
#define BIORBD_MATLAB_INVERSE_KINEMATICS_EKF_IMU_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"
#include "RigidBody/KalmanReconsIMU.h"

void Matlab_setEKF_IMU(int nlhs, mxArray *plhs[],
                       int nrhs, const mxArray*prhs[] )
{
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 2, 5,
                               "2 arguments are required (+3 optional) where the 2nd is the handler on the model, the optional 3th is acquisition frequency [default 100Hz], 4th is noise factor [default 5e-3] and 5th is error factor [default 1e-10]");

    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // S'assurer que la personne recueille l'acces au filtre de Kalman
    if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:yprime:maxlhs",
                           "You must catch the pointer address!");
    }

    double freq=100, noiseF=5e-3, errorF=1e-10;
    if (nrhs >= 5) {
        errorF = getDouble(prhs,4,"Error Factor");
    }
    if (nrhs >= 4) {
        noiseF = getDouble(prhs,3,"Noise Factor");
    }
    if (nrhs >= 3) {
        freq = getDouble(prhs,2,"Acquisition Frequency");
    }
    biorbd::rigidbody::KalmanParam kParams(freq, noiseF, errorF);

    // Créer un filtre de Kalman
    try {
        plhs[0] = convertPtr2Mat<biorbd::rigidbody::KalmanReconsIMU>
                  (new biorbd::rigidbody::KalmanReconsIMU(*model, kParams));
    } catch (std::string m) {
        mexErrMsgTxt(m.c_str());
    }

    return;
}
void Matlab_delEKF_IMU(int nlhs, mxArray *[],
                       int nrhs, const mxArray*prhs[] )
{
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 2, 2,
                               "2 arguments are required where the 2nd is the handler on the kalman filter");

    // Destroy the C++ object
    destroyObject<biorbd::rigidbody::KalmanReconsIMU>(prhs[1]);
    // Warn if other commands were ignored
    if (nlhs != 0 || nrhs != 2) {
        mexWarnMsgTxt("Delete: Unexpected output arguments ignored.");
    }
    return;
}


void Matlab_inverseKinematicsEKF_IMUstep( int , mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
{
    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 4, 5,
                               "4 arguments are required (+1 optional) where the 2nd is the handler on the model, 3rd is the handler on kalman filter info, 4th is the 3x3xNxT or 4x4xNxT IMU hypermatrix, the optional 5th is the initial guess for Q [default 0], 5th is acquisition frequency [default 100Hz]");

    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF
    unsigned int nQdot = model->nbQdot(); // Get the number of DoF
    unsigned int nQddot = model->nbQddot(); // Get the number of DoF

    // Recevoir le kalman
    biorbd::rigidbody::KalmanReconsIMU * kalman =
        convertMat2Ptr<biorbd::rigidbody::KalmanReconsIMU>(prhs[2]);

    // Recevoir la matrice des markers (Ne traite que le premier frame)
    std::vector<std::vector<biorbd::rigidbody::IMU>> imusOverTime =
                getParameterAllIMUs(prhs,3);
    std::vector<biorbd::rigidbody::IMU> imus = imusOverTime[0];

    // Si c'est le premier frame recevoir Qinit

    if (kalman->first() && nrhs >= 5) {
        biorbd::rigidbody::GeneralizedCoordinates Qinit(*getParameterQ(prhs, 4,
                nQ).begin());
        kalman->setInitState(&Qinit);
    }

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix(nQ , 1, mxREAL); // Q
    plhs[1] = mxCreateDoubleMatrix(nQdot , 1, mxREAL); // QDot
    plhs[2] = mxCreateDoubleMatrix(nQddot , 1, mxREAL); // QDDot
    double *q = mxGetPr(plhs[0]);
    double *qdot = mxGetPr(plhs[1]);
    double *qddot = mxGetPr(plhs[2]);

    // Faire la cinématique inverse a chaque instant
    biorbd::rigidbody::GeneralizedCoordinates Q(nQ);
    biorbd::rigidbody::GeneralizedVelocity QDot(nQdot);
    biorbd::rigidbody::GeneralizedAcceleration QDDot(nQddot);

    // Faire la cinématique inverse
    kalman->reconstructFrame(*model, imus, &Q, &QDot, &QDDot);

    // Remplir la variable de sortie
    for (unsigned int j=0; j<nQ; ++j) {
        q[j] = Q(j);
    }

    for (unsigned int j=0; j<nQdot; ++j) {
        qdot[j] = QDot(j);
    }

    for (unsigned int j=0; j<nQddot; ++j) {
        qddot[j] = QDDot(j);
    }

    return;
}

void Matlab_inverseKinematicsEKF_IMUallInOneCall( int, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 7,
                               "4 arguments are required (+5 optional) where the 2nd is the handler on the model, 3th is the 3x3xNxT or 4x4xNxT IMU hypermatrix, the optional 4th is the initial guess for Q [default 0], 5th is acquisition frequency [default 100Hz], 6th is noise factor [default 5e-3] and 7th is error factor [default 1e-10]");

    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF
    unsigned int nQdot = model->nbQdot(); // Get the number of DoF
    unsigned int nQddot = model->nbQddot(); // Get the number of DoF

    // Recevoir les paramètres du filtre
    double freq=100, noiseF=5e-3, errorF=1e-10;
    if (nrhs >= 7) {
        errorF = getDouble(prhs,6,"Error Factor");
    }
    if (nrhs >= 6) {
        noiseF = getDouble(prhs,5,"Noise Factor");
    }
    if (nrhs >= 5) {
        freq = getDouble(prhs,4,"Acquisition Frequency");
    }

    // Créer un filtre de Kalman
    biorbd::rigidbody::KalmanReconsIMU kalman(*model,
            biorbd::rigidbody::KalmanParam(freq, noiseF, errorF));

    // Recevoir la matrice des imus
    std::vector<std::vector<biorbd::rigidbody::IMU>> imusOverTime =
                getParameterAllIMUs(prhs,2);
    unsigned int nFrames(static_cast<unsigned int>(imusOverTime.size()));

    // Recevoir Qinit
    if (kalman.first() && nrhs >= 4) {
        biorbd::rigidbody::GeneralizedCoordinates Qinit(*getParameterQ(prhs, 3,
                nQ).begin());
        kalman.setInitState(&Qinit);
    }

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix(nQ , nFrames, mxREAL); // Q
    plhs[1] = mxCreateDoubleMatrix(nQdot , nFrames, mxREAL); // QDot
    plhs[2] = mxCreateDoubleMatrix(nQddot , nFrames, mxREAL); // QDDot
    double *q = mxGetPr(plhs[0]);
    double *qdot = mxGetPr(plhs[1]);
    double *qddot = mxGetPr(plhs[2]);

    unsigned int cmp(0);
    for (unsigned int i=0; i<nFrames; ++i) {
        // Faire la cinématique inverse a chaque instant
        biorbd::rigidbody::GeneralizedCoordinates Q(nQ);
        biorbd::rigidbody::GeneralizedVelocity QDot(nQdot);
        biorbd::rigidbody::GeneralizedAcceleration QDDot(nQddot);

        // Faire la cinématique inverse
        kalman.reconstructFrame(*model, *(imusOverTime.begin()+i), &Q, &QDot, &QDDot);

        // Remplir la variable de sortie
        for (unsigned int j=0; j<nQ; ++j) {
            q[cmp] = Q(j);
            qdot[cmp] = QDot(j);
            qddot[cmp] = QDDot(j);
            ++cmp;
        }
    }

    return;
}

#endif // BIORBD_MATLAB_INVERSE_KINEMATICS_EKF_IMU_H
