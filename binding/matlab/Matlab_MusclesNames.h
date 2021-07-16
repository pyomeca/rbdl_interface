#ifndef BIORBD_MATLAB_MUSCLES_NAMES_H
#define BIORBD_MATLAB_MUSCLES_NAMES_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"
#include "Muscles/MuscleGroup.h"
#include "Muscles/Muscle.h"
#include "Muscles/PathModifiers.h"

void Matlab_MusclesNames( int, mxArray *plhs[],
                          int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 2,
                               "2 arguments are required where the 2nd is the handler on the model");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Sortie des noms
    plhs[0] = mxCreateCellMatrix(model->nbMuscleGroups(),
                                 1); // Stockage des noms de groupe
    plhs[1] = mxCreateCellMatrix(model->nbMuscleGroups(),
                                 1); // Stockage des noms de muscles
    plhs[2] = mxCreateCellMatrix(model->nbMuscleGroups(),
                                 1); // Stockage des noms de viapoints

    // Stocker chaque valeur
    // Pour chaque groupe musculaire
    for (unsigned int i=0; i<model->nbMuscleGroups(); ++i) {
        // Créer une matrice de cellule pour recueillir les noms (a stocker dans la plus grosse matrice)
        mxArray *muscleNames = mxCreateCellMatrix(model->muscleGroup(i).nbMuscles(), 1);
        // Créer une matrice de cellule pour recueillir les noms (a stocker dans la plus grosse matrice)
        mxArray *viaNamesByMuscles = mxCreateCellMatrix(model->muscleGroup(
                                         i).nbMuscles(), 1);


        // Pour chaque muscle
        for (unsigned int j=0; j<model->muscleGroup(i).nbMuscles(); ++j) {
            mxArray * name = mxCreateString(model->muscleGroup(i).muscle(
                                                j).name().c_str()); // Recueillir le nom du muscle
            mxSetCell(muscleNames, j, name); // Les mettres une cellule

            // Pour chaque via points / wraping
            mxArray *viaNames = mxCreateCellMatrix(model->muscleGroup(i).muscle(
                    j).pathModifier().nbObjects(),1);
            for (unsigned int k=0;
                    k<model->muscleGroup(i).muscle(j).pathModifier().nbObjects(); ++k) {
                mxArray * viaName = mxCreateString(model->muscleGroup(i).muscle(
                                                       j).pathModifier().object(
                                                       k).name().c_str()); // Recueillir le nom du muscle
                mxSetCell(viaNames, k, viaName); // Les mettres une cellule
            }
            mxSetCell(viaNamesByMuscles, j,
                      viaNames); // Mettre la matrice de viapoints dans une matrice
        }
        // Mettre les groupNames dans la variable de sortie
        mxArray *groupNames = mxCreateString (model->muscleGroup(
                i).name().c_str()); // Recueillir le nom du groupe
        mxSetCell(plhs[0],i,groupNames);

        // Mettre les noms de muscles pour ce groupe
        mxSetCell(plhs[1],i,muscleNames);

        // Mettre les noms de via points pour ce groupe
        mxSetCell(plhs[2],i,viaNamesByMuscles);
    }

    return;
}

#endif // BIORBD_MATLAB_MUSCLES_NAMES_H
