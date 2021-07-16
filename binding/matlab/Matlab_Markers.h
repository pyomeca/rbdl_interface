#ifndef BIORBD_MATLAB_MARKERS_H
#define BIORBD_MATLAB_MARKERS_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_Markers( int, mxArray *plhs[],
                     int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 5,
                               "3 arguments are required [+2 optional] where the 2nd is the handler on the model, 3rd is the Q and 4th is the wanted markerType to be return ('all', 'technical' or anatomical') and 5th if you want to remove axes as specified in the model file [default = true]");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF

    // Recevoir Q
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> Q = getParameterQ(prhs,
            2, nQ);

    bool removeAxes(true);
    if (nrhs >= 5) {
        removeAxes = getBool(prhs, 4);
    }

    // Récupérer les marqueurs selon que l'on veut tous ou seulement anatomiques ou techniques
    unsigned int nMarkers(0); // Nombre de marqueurs
    std::vector<std::vector<biorbd::rigidbody::NodeSegment>>
            markers_tp; // récupérer les marqueurs
    if (nrhs >= 4) {
        biorbd::utils::String type(getString(prhs,3));
        if (!type.tolower().compare("all")) {
            nMarkers = model->nbMarkers();
            for (std::vector<biorbd::rigidbody::GeneralizedCoordinates>::iterator Q_it =
                        Q.begin(); Q_it!=Q.end(); ++Q_it) {
                markers_tp.push_back(model->markers(*Q_it, removeAxes));
            }
        } else if (!type.tolower().compare("anatomical")) {
            nMarkers = model->nbAnatomicalMarkers();
            for (std::vector<biorbd::rigidbody::GeneralizedCoordinates>::iterator Q_it =
                        Q.begin(); Q_it!=Q.end(); ++Q_it) {
                markers_tp.push_back(model->anatomicalMarkers(*Q_it, removeAxes));
            }
        } else if (!type.tolower().compare("technical")) {
            nMarkers = model->nbTechnicalMarkers();
            for (std::vector<biorbd::rigidbody::GeneralizedCoordinates>::iterator Q_it =
                        Q.begin(); Q_it!=Q.end(); ++Q_it) {
                markers_tp.push_back(model->technicalMarkers(*Q_it, removeAxes));
            }
        } else {
            std::ostringstream msg;
            msg << "Wrong type for markers!";
            mexErrMsgTxt(msg.str().c_str());
        }

    } else {
        nMarkers = model->nbMarkers();
        for (std::vector<biorbd::rigidbody::GeneralizedCoordinates>::iterator Q_it =
                    Q.begin(); Q_it!=Q.end(); ++Q_it) {
            markers_tp.push_back(model->markers(*Q_it, removeAxes));
        }
    }

    // Create a matrix for the return argument
    mwSize dims[3];
    dims[0] = 3;
    dims[1] = nMarkers;
    dims[2] = markers_tp.size();

    plhs[0] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
    double *markers = mxGetPr(plhs[0]);

    // Remplir le output
    unsigned int cmp(0);
    for (std::vector<std::vector<biorbd::rigidbody::NodeSegment>>::iterator
            markers_it = markers_tp.begin();
            markers_it!=markers_tp.end(); ++markers_it) {
        std::vector<biorbd::rigidbody::NodeSegment>::iterator it=(*markers_it).begin();
        for (unsigned int i=0; (it+i)!=(*markers_it).end(); ++i) {
            markers[cmp+0] = (*(it+i))(0);
            markers[cmp+1] = (*(it+i))(1);
            markers[cmp+2] = (*(it+i))(2);
            cmp += 3;
        }
    }

    return;
}

#endif // BIORBD_MATLAB_MARKERS_H
