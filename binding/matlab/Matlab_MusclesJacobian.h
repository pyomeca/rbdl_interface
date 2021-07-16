#ifndef BIORBD_MATLAB_MUSCLES_JACOBIAN_H
#define BIORBD_MATLAB_MUSCLES_JACOBIAN_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"
#include "Utils/Matrix.h"
#include "Muscles/MuscleGroup.h"
#include "Muscles/Muscle.h"
#include "Muscles/Geometry.h"

void Matlab_MusclesJacobian( int, mxArray *plhs[],
                             int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entrée
    checkNombreInputParametres(nrhs, 3, 3,
                               "3 arguments are required where the 2nd is the handler on the model and 3rd is the Q");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF

    // Recevoir Q
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> Q = getParameterQ(prhs,
            2, nQ);

    // Cellules de sortie
    mwSize dims[4];
    dims[0] = 3*2;
    dims[1] = nQ;
    dims[2] = model->nbMuscleTotal();
    dims[3] = Q.size();
    plhs[0] = mxCreateCellMatrix(model->nbMuscleTotal(), Q.size());

    // Aller chercher les valeurs
    unsigned int cmpCell(0);
    for (unsigned int iQ=0; iQ<Q.size(); ++iQ) {
        int updateKin(2);
        for (unsigned int i=0; i<model->nbMuscleGroups(); ++i)
            for (unsigned int j=0; j<model->muscleGroup(i).nbMuscles(); ++j) {
                // Recueillir toutes les longueurs musculaire
                model->muscleGroup(i).muscle(j).updateOrientations(*model, Q[iQ], updateKin);
                updateKin = 1;
                Eigen::MatrixXd tp = model->muscleGroup(i).muscle(j).position().jacobian();

                // Transférer les données dans une matrice matlab
                mxArray* jacoByMus = mxCreateDoubleMatrix(static_cast<mwSize>(tp.rows()),
                                     static_cast<mwSize>(tp.cols()), mxREAL);
                double* ptrJacoByMus = mxGetPr(jacoByMus);
                unsigned int cmpElement(0);
                for (unsigned int k2=0; k2<nQ; ++k2) {
                    for (unsigned int k1=0; k1<tp.rows(); ++k1) {
                        ptrJacoByMus[cmpElement] = tp(k1,k2);
                        ++cmpElement;
                    }
                }

                mxSetCell(plhs[0],cmpCell,jacoByMus);
                ++cmpCell;
            }
    }

    return;
}

#endif // BIORBD_MATLAB_MUSCLES_JACOBIAN_H
