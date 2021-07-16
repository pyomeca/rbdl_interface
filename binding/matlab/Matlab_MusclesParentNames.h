#ifndef BIORBD_MATLAB_MUSCLES_PARENT_NAMES_H
#define BIORBD_MATLAB_MUSCLES_PARENT_NAMES_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"
#include "Muscles/MuscleGroup.h"
#include "Muscles/Muscle.h"
#include "Muscles/Geometry.h"
#include "Muscles/PathModifiers.h"

void Matlab_MusclesParentNames( int, mxArray *plhs[],
                                int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2,
                               "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Sortie des noms
    plhs[0] = mxCreateCellMatrix(model->nbMuscleGroups(),
                                 1); // Stockage des noms de viapoints

    // Stocker chaque valeur
    // Pour chaque groupe musculaire
    for (unsigned int i=0; i<model->nbMuscleGroups(); ++i) {
        // Créer une matrice de cellule pour recueillir les noms (a stocker dans la plus grosse matrice)
        mxArray *viaNamesByMuscles = mxCreateCellMatrix(model->muscleGroup(
                                         i).nbMuscles(), 1);
        // Pour chaque muscle
        for (unsigned int j=0; j<model->muscleGroup(i).nbMuscles(); ++j) {
            // Pour chaque via points / wraping
            mxArray *viaNames = mxCreateCellMatrix(model->muscleGroup(i).muscle(
                    j).pathModifier().nbObjects()+2,
                                                   1);// +2 pour origine et insertion

            mxArray * nameOrigin = mxCreateString(model->muscleGroup(i).muscle(
                    j).position().originInLocal().parent().c_str()); // Origine
            mxSetCell(viaNames, 0, nameOrigin);
            for (unsigned int k=0;
                    k<model->muscleGroup(i).muscle(j).pathModifier().nbObjects(); ++k) {
                mxArray * viaName = mxCreateString(model->muscleGroup(i).muscle(
                                                       j).pathModifier().object(
                                                       k).parent().c_str()); // Recueillir le nom du muscle
                mxSetCell(viaNames, k+1,
                          viaName); // Les mettres une cellule (+1 pour skipper origin)
            }
            mxArray * nameInsertion = mxCreateString(model->muscleGroup(i).muscle(
                                          j).position().insertionInLocal().parent().c_str()); // Insertion
            mxSetCell(viaNames, model->muscleGroup(i).muscle(j).pathModifier().nbObjects()
                      +1, nameInsertion);

            mxSetCell(viaNamesByMuscles, j,
                      viaNames); // Mettre la matrice de viapoints dans une matrice
        }
        // Mettre les noms de via points pour ce groupe
        mxSetCell(plhs[0],i,viaNamesByMuscles);
    }

    return;
}

#endif // BIORBD_MATLAB_MUSCLES_PARENT_NAMES_H
