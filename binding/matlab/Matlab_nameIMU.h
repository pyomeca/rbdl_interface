#ifndef BIORBD_MATLAB_NAME_IMU_H
#define BIORBD_MATLAB_NAME_IMU_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_nameIMU( int, mxArray *plhs[],
                     int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2,
                               "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Trouver ou sont les marqueurs
    std::vector<biorbd::utils::String> allIMU(model->IMUsNames());

    // Create a matrix for the return argument
    plhs[0] = mxCreateCellMatrix( allIMU.size(), 1);
    for (unsigned int i_bone=0; i_bone<allIMU.size(); ++i_bone) {
        mxArray * imu_out_tp = mxCreateString((*(allIMU.begin()+i_bone)).c_str());
        mxSetCell(plhs[0],i_bone,imu_out_tp);
    }

    return;
}

void Matlab_nameTechnicalIMU( int, mxArray *plhs[],
                              int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2,
                               "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Trouver ou sont les marqueurs
    std::vector<biorbd::utils::String> allIMU(model->technicalIMUsNames());

    // Create a matrix for the return argument
    plhs[0] = mxCreateCellMatrix( allIMU.size(), 1);
    for (unsigned int i_bone=0; i_bone<allIMU.size(); ++i_bone) {
        mxArray * imu_out_tp = mxCreateString((*(allIMU.begin()+i_bone)).c_str());
        mxSetCell(plhs[0],i_bone,imu_out_tp);
    }

    return;
}

void Matlab_nameAnatomicalIMU( int, mxArray *plhs[],
                               int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2,
                               "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Trouver ou sont les marqueurs
    std::vector<biorbd::utils::String> allIMU(model->anatomicalIMUsNames());

    // Create a matrix for the return argument
    plhs[0] = mxCreateCellMatrix( allIMU.size(), 1);
    for (unsigned int i_bone=0; i_bone<allIMU.size(); ++i_bone) {
        mxArray * imu_out_tp = mxCreateString((*(allIMU.begin()+i_bone)).c_str());
        mxSetCell(plhs[0],i_bone,imu_out_tp);
    }

    return;
}

#endif // BIORBD_MATLAB_NAME_IMU_H
