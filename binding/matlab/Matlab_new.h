#ifndef BIORBD_MATLAB_NEW_H
#define BIORBD_MATLAB_NEW_H

#include <mex.h>
#include "BiorbdModel.h"
#include "ModelReader.h"
#include "class_handle.h"

#include "Utils/String.h"

void Matlab_new( int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray*prhs[] )
{
    if (nrhs != 2) {
        mexErrMsgIdAndTxt( "MATLAB:yprime:invalidNumInputs",
                           "2 arguments are required where the 2nd is the path to the model");
    }

    // Get the path of the model
    if (!mxIsChar(prhs[1]) || (mxGetM(prhs[1]) != 1 ) )  {
        mexErrMsgIdAndTxt( "MATLAB:mxmalloc:invalidInput",
                           "Input argument 2 must be a file path string.");
    }
    if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:yprime:maxlhs",
                           "You must catch the pointer address!");
    }
    char *buf = mxArrayToString(prhs[1]);
    biorbd::utils::String filepath(buf); // Copier le cstring dans un std::string

    // Loader le modèle musculosquelettique
    // Definition des variables globales du modele
    try {
        plhs[0] = convertPtr2Mat<biorbd::Model>(
                      new biorbd::Model(biorbd::Reader::readModelFile(filepath)));
    } catch (std::string m) {
        mexErrMsgTxt(m.c_str());
    }

    return;
}
#endif // BIORBD_MATLAB_NEW_H
