#ifndef BIORBD_MATLAB_SEGMENT_MARKERS_H
#define BIORBD_MATLAB_SEGMENT_MARKERS_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_segmentMarkers( int, mxArray *plhs[],
                            int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 3, 5,
                               "3 arguments are required (+2 optional) where the 2nd is the handler on the model, 3rd is the Q, 4th if you want to remove axes as specified in the model file [default = true] and 5th optional is a specific segment index");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF

    // Recevoir Q
    biorbd::rigidbody::GeneralizedCoordinates Q = *getParameterQ(prhs, 2,
            nQ).begin();

    bool removeAxes(true);
    if (nrhs>=4) {
        removeAxes = getBool(prhs, 3);
    }

    // Recevoir l'index
    int idx(-1);
    if (nrhs >= 5) {
        idx = getInteger(prhs,4)-1;    // -1 afin que le segment 1 soit le root
    }


    if ( idx==-1) { // Si on a demande tous les segments
        // Trouver ou sont les marqueurs
        std::vector<std::vector<biorbd::rigidbody::NodeSegment>> allMarkers;
        for (unsigned int i=0; i<model->nbSegment(); ++i)    {
            std::vector<biorbd::rigidbody::NodeSegment> markers_tp = model->segmentMarkers(
                        Q, i, removeAxes);
            allMarkers.push_back(markers_tp);
        }
        // Create a matrix for the return argument
        plhs[0] = mxCreateCellMatrix( allMarkers.size(), 1);
        for (unsigned int i_bone=0; i_bone<allMarkers.size(); ++i_bone) {
            mxArray *markers_out_tp = mxCreateDoubleMatrix( 3,
                                      (*(allMarkers.begin()+i_bone)).size(), mxREAL);
            double *markers = mxGetPr(markers_out_tp);

            // Remplir le output
            std::vector<biorbd::rigidbody::NodeSegment>::iterator it=(*
                    (allMarkers.begin()+i_bone)).begin();
            for (unsigned int i=0; (it+i)!=(*(allMarkers.begin()+i_bone)).end(); ++i) {
                markers[i*3] = (*(it+i))(0);
                markers[i*3+1] = (*(it+i))(1);
                markers[i*3+2] = (*(it+i))(2);
            }
            mxSetCell(plhs[0],i_bone,markers_out_tp);
        }
        return;

    } else { // Si on a demande un segment precis
        std::vector<biorbd::rigidbody::NodeSegment> markers_tp =
            model->segmentMarkers(Q, static_cast<unsigned int>(idx), removeAxes);

        // Create a matrix for the return argument
        plhs[0] = mxCreateDoubleMatrix(3, markers_tp.size(), mxREAL);
        double *markers = mxGetPr(plhs[0]);

        // Remplir le output
        std::vector<biorbd::rigidbody::NodeSegment>::iterator it=markers_tp.begin();
        for (unsigned int i=0; (it+i)!=markers_tp.end(); ++i) {
            markers[i*3] = (*(it+i))(0);
            markers[i*3+1] = (*(it+i))(1);
            markers[i*3+2] = (*(it+i))(2);
        }
        return;
    }
}

#endif // BIORBD_MATLAB_SEGMENT_MARKERS_H
