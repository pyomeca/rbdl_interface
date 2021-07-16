#ifndef BIORBD_MATLAB_NAME_DOF_H
#define BIORBD_MATLAB_NAME_DOF_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_nameDof( int, mxArray *plhs[],
                     int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2,
                               "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    unsigned int nQ = model->nbQ(); // Get the number of DoF

    // Sortie des noms
    plhs[0] = mxCreateCellMatrix(nQ, 1); // Stockage des noms de groupe

    // Stocker chaque valeur
    unsigned int cmp(0);
    for (unsigned int i=0; i<model->nbSegment(); ++i) {
        for (unsigned int j=0; j<model->segment(i).nbDof(); ++j) {
            mxArray *nomDof = mxCreateString ((model->segment(i).name() + "_" +
                                               model->segment(i).nameDof(
                                                   j)).c_str()); // Recueillir le nom
            mxSetCell(plhs[0],cmp,nomDof); // Mettre les noms dans la variable de sortie
            ++cmp;
        }
    }

    return;
}

#endif // BIORBD_MATLAB_NAME_DOF_H
