#ifndef BIORBD_MATLAB_CHANGE_SHAPE_FACTOR_H
#define BIORBD_MATLAB_CHANGE_SHAPE_FACTOR_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"
#include "Muscles/Muscle.h"
#include "Muscles/MuscleGroup.h"

void Matlab_ChangeShapeFactors( int, mxArray *[],
                                int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3,
                               "3 arguments are required where the 2nd is the handler on the model, 3rd is the shape factor for each muscle");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Recevoir les shape factors
    std::vector<double> shapeFactors = getDoubleArray(prhs, 2);
    if (shapeFactors.size() != model->nbMuscleTotal()) {
        std::ostringstream msg;
        msg << "Length of vector of shape factors should match the number of muscles ("
            << model->nbMuscleTotal() << ")";
        mexErrMsgTxt(msg.str().c_str());
    }

    // Aller changer les valeurs
    unsigned int cmp(0);
    for (unsigned int i=0; i<model->nbMuscleGroups(); ++i)
        for (unsigned int j=0; j<model->muscleGroup(i).nbMuscles(); ++j) {
            // Recueillir shape factor
            dynamic_cast<biorbd::muscles::StateDynamicsBuchanan&>( model->muscleGroup(
                        i).muscle(j).state() ).shapeFactor(
                            shapeFactors[cmp]);
            ++cmp;
        }

    return;
}

#endif // BIORBD_MATLAB_CHANGE_SHAPE_FACTOR_H
