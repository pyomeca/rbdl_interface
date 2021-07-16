#ifndef BIORBD_MATLAB_NAME_MARKERS_H
#define BIORBD_MATLAB_NAME_MARKERS_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_nameMarkers( int, mxArray *plhs[],
                         int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2,
                               "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Trouver ou sont les marqueurs
    std::vector<biorbd::utils::String> allMarkers(model->markerNames());

    // Create a matrix for the return argument
    plhs[0] = mxCreateCellMatrix( allMarkers.size(), 1);
    for (unsigned int i_bone=0; i_bone<allMarkers.size(); ++i_bone) {
        mxArray * markers_out_tp = mxCreateString((*(allMarkers.begin()
                                   +i_bone)).c_str());
        mxSetCell(plhs[0],i_bone,markers_out_tp);
    }

    return;
}

void Matlab_nameTechnicalMarkers( int, mxArray *plhs[],
                                  int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2,
                               "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Trouver ou sont les marqueurs
    std::vector<biorbd::utils::String> allMarkers(model->technicalMarkerNames());

    // Create a matrix for the return argument
    plhs[0] = mxCreateCellMatrix( allMarkers.size(), 1);
    for (unsigned int i_bone=0; i_bone<allMarkers.size(); ++i_bone) {
        mxArray * markers_out_tp = mxCreateString((*(allMarkers.begin()
                                   +i_bone)).c_str());
        mxSetCell(plhs[0],i_bone,markers_out_tp);
    }

    return;
}

void Matlab_nameAnatomicalMarkers( int, mxArray *plhs[],
                                   int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2,
                               "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Trouver ou sont les marqueurs
    std::vector<biorbd::utils::String> allMarkers(model->anatomicalMarkerNames());

    // Create a matrix for the return argument
    plhs[0] = mxCreateCellMatrix( allMarkers.size(), 1);
    for (unsigned int i_bone=0; i_bone<allMarkers.size(); ++i_bone) {
        mxArray * markers_out_tp = mxCreateString((*(allMarkers.begin()
                                   +i_bone)).c_str());
        mxSetCell(plhs[0],i_bone,markers_out_tp);
    }

    return;
}

#endif // BIORBD_MATLAB_NAME_MARKERS_H
