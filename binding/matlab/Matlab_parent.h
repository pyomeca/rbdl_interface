#ifndef BIORBD_MATLAB_PARENT_H
#define BIORBD_MATLAB_PARENT_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_parent( int, mxArray *plhs[],
                    int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3,
                               "3 arguments are required where the 2nd is the handler on the model and the 3rd is the name of the segment");
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Create a matrix for the return argument
    plhs[0] = mxCreateDoubleMatrix( 1, 1, mxREAL);

    // Quel segment
    unsigned int idx = static_cast<unsigned int>(getInteger(prhs, 2, "index"));

    if (idx<1) {
        std::ostringstream msg;
        msg << "Segment index must be 1 or higher.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    } else if (idx>model->nbSegment()) {
        std::ostringstream msg;
        msg << "Segment index must not be higher than number of segment.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }

    // Sortie du nom du segment parent
    plhs[0] = mxCreateString ( model->segment(idx
                               -1).name().c_str()); // Recueillir le nom

    return;
}

#endif // BIORBD_MATLAB_PARENT_H
