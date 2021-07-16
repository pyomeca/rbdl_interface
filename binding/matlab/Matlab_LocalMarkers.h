#ifndef BIORBD_MATLAB_LOCAL_MARKERS_H
#define BIORBD_MATLAB_LOCAL_MARKERS_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_LocalMarkers( int, mxArray *plhs[],
                          int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 2, 4,
                               "2 arguments are required [+2 optional] where the 2nd is the handler on the model, 3rd is the wanted markerType to be return ('all' [default], 'technical' or anatomical') and 4th if you want to remove axes as specified in the model file [default = true]");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Gestion du type
    biorbd::utils::String type("all");
    if (nrhs >= 3) {
        type = getString(prhs,2);
    }
    bool removeAxes(true);
    if (nrhs >= 4) {
        removeAxes = getBool(prhs, 3);
    }

    // Récupérer les marqueurs selon que l'on veut tous ou seulement anatomiques ou techniques
    unsigned int nMarkers(0); // Nombre de marqueurs
    std::vector<biorbd::rigidbody::NodeSegment>
    markers_tp; // récupérer les marqueurs
    if (!type.tolower().compare("all")) {
        nMarkers = model->nbMarkers();
        markers_tp = model->biorbd::rigidbody::Markers::markers(removeAxes);
    } else if (!type.tolower().compare("anatomical")) {
        nMarkers = model->nbAnatomicalMarkers();
        markers_tp = model->biorbd::rigidbody::Markers::anatomicalMarkers(removeAxes);
    } else if (!type.tolower().compare("technical")) {
        nMarkers = model->nbTechnicalMarkers();
        markers_tp = model->biorbd::rigidbody::Markers::technicalMarkers(removeAxes);
    } else {
        std::ostringstream msg;
        msg << "Wrong type for markers!";
        mexErrMsgTxt(msg.str().c_str());
    }

    // Create a matrix for the return argument
    mwSize dims[2];
    dims[0] = 3;
    dims[1] = nMarkers;

    plhs[0] = mxCreateNumericArray(2, dims, mxDOUBLE_CLASS, mxREAL);
    double *markers = mxGetPr(plhs[0]);

    // Remplir le output
    unsigned int cmp(0);
    std::vector<biorbd::rigidbody::NodeSegment>::iterator it=markers_tp.begin();
    for (unsigned int i=0; (it+i)!=markers_tp.end(); ++i) {
        markers[cmp+0] = (*(it+i))(0);
        markers[cmp+1] = (*(it+i))(1);
        markers[cmp+2] = (*(it+i))(2);
        cmp += 3;
    }

    return;
}

#endif // BIORBD_MATLAB_LOCAL_MARKERS_H
