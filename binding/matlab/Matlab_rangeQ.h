#ifndef BIORBD_MATLAB_RANGE_Q_H
#define BIORBD_MATLAB_RANGE_Q_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"
#include "Utils/Range.h"

void Matlab_rangeQ(int, mxArray *plhs[],
                   int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 2, 2,
                               "2 arguments are required where the 2nd is the handler on the model");
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nbSegments = model->nbSegment();

    mwSize outSize[2] = {nbSegments, 1};
    plhs[0] = mxCreateCellArray(2, outSize);

    // Stocker chaque valeur
    for (unsigned int i=0; i<nbSegments; ++i) {
        const auto& ranges = model->segment(i).QRanges();
        mxArray* rangeSegment = mxCreateDoubleMatrix(2, ranges.size(),
                                mxREAL); // Stockage des noms de groupe
        double *rangeSegmentDouble = mxGetPr(rangeSegment);
        for (unsigned int j=0; j<ranges.size(); ++j) {
            rangeSegmentDouble[j*2 + 0] = ranges[j].min();
            rangeSegmentDouble[j*2 + 1] = ranges[j].max();
        }
        mxSetCell(plhs[0],i,rangeSegment);
    }
    return;
}

#endif // BIORBD_MATLAB_RANGE_Q_H
