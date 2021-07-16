#ifndef BIORBD_MATLAB_MUSCLES_POINTS_H
#define BIORBD_MATLAB_MUSCLES_POINTS_H

#include <memory>
#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"
#include "Utils/Vector3d.h"
#include "Muscles/WrappingHalfCylinder.h"
#include "Muscles/MuscleGroup.h"
#include "Muscles/Muscle.h"
#include "Muscles/PathModifiers.h"

void Matlab_MusclesPoints( int nlhs, mxArray *plhs[],
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
    plhs[0] = mxCreateCellMatrix(model->nbMuscleTotal(), Q.size());
    // Utilisable que si nlhs >= 2
    std::vector<biorbd::utils::NODE_TYPE> wrap_type; // forme du wrapping
    std::vector<biorbd::utils::RotoTrans> wrap_RT; // orientation du wrapping
    std::vector<double> wrap_dim1; // dimension du wrapping
    std::vector<double> wrap_dim2; // dimension deux du wrapping


    // Stocker les position des origines/insertion/viapoints
    unsigned int cmp(0);
    for (unsigned int iQ=0; iQ<Q.size(); ++iQ)
        for (unsigned int i=0; i<model->nbMuscleGroups(); ++i)
            for (unsigned int j=0; j<model->muscleGroup(i).nbMuscles(); ++j) {
                model->UpdateKinematicsCustom(&Q[iQ]);

                // Recueillir tout le chemin musculaire
                std::vector<biorbd::utils::Vector3d> via(model->muscleGroup(i).muscle(
                            j).musclesPointsInGlobal());

                // Si le nombre de wrap est > 0, c'est qu'il n'y a pas de viapoints et il n'y a qu'UN wrap
                if (model->muscleGroup(i).muscle(j).pathModifier().nbWraps() > 0) {
                    const biorbd::muscles::WrappingObject& wrappingObject(
                        dynamic_cast<const biorbd::muscles::WrappingObject&>(
                            model->muscleGroup(0).muscle(0).pathModifier().object(0)));

                    // De quel type
                    biorbd::utils::NODE_TYPE type(wrappingObject.typeOfNode());
                    wrap_type.push_back(type);

                    // Dans quelle orientation
                    wrap_RT.push_back(wrappingObject.RT());

                    // Quel est sa dimension
                    if (type == biorbd::utils::NODE_TYPE::WRAPPING_HALF_CYLINDER) {
                        const biorbd::muscles::WrappingHalfCylinder& cylinder(
                            dynamic_cast<const biorbd::muscles::WrappingHalfCylinder&>
                            (wrappingObject));
                        wrap_dim1.push_back(cylinder.radius());
                        wrap_dim2.push_back(cylinder.length());
                    }
                }


                // transférer toutes les valeurs dans une variable temporaire
                mxArray *tp = mxCreateDoubleMatrix(3,via.size(),mxREAL);
                double *ptrTp = mxGetPr(tp);
                for (unsigned int k=0; k<via.size(); ++k) {
                    ptrTp[k*3]   = (*(via.begin()+k))[0];
                    ptrTp[k*3+1] = (*(via.begin()+k))[1];
                    ptrTp[k*3+2] = (*(via.begin()+k))[2];
                }

                mxSetCell(plhs[0],cmp,tp);
                ++cmp;
            }

    if (nlhs >= 2) { // Sortir la déclaration des wrapping s'il y a lieu
        mwSize dims[3];
        unsigned int nWrap(static_cast<unsigned int>(wrap_RT.size()/Q.size()));
        dims[0] = nWrap;
        dims[1] = 4;
        dims[2] = Q.size();
        plhs[1] = mxCreateCellArray(3, dims);
        unsigned int cmpWrap(0);
        unsigned int cmpTime(0);

        for (unsigned int i=0; i<wrap_RT.size(); ++i) {
            // Transcrire les formes dans un tableau matlab
            mxArray *tp_forme = mxCreateString( biorbd::utils::NODE_TYPE_toStr(
                                                    wrap_type[i]) );

            // Transcrire les valeurs RT dans un tableau matlab
            mxArray *tp_RT = mxCreateDoubleMatrix(4,4,mxREAL);
            double *RT = mxGetPr(tp_RT);
            int cmp(0);
            for (unsigned int j=0; j<4; ++j)
                for (unsigned int k=0; k<4; ++k) {
                    RT[cmp++] = wrap_RT[i](k, j);
                }

            // Transcrire les valeurs de dimension1
            mxArray *tp_dim1 = mxCreateDoubleMatrix(1,1,mxREAL);
            double *dim1 = mxGetPr(tp_dim1);
            dim1[0] = *(wrap_dim1.begin()+i);
            // Transcrire les valeurs de dimension2
            mxArray *tp_dim2 = mxCreateDoubleMatrix(1,1,mxREAL);
            double *dim2 = mxGetPr(tp_dim2);
            dim2[0] = *(wrap_dim2.begin()+i);

            // Stocker les informations dans la variable de sortie
            mxSetCell(plhs[1],(nWrap*4)*cmpTime+(cmpWrap+0*nWrap),tp_forme);
            mxSetCell(plhs[1],(nWrap*4)*cmpTime+(cmpWrap+1*nWrap),tp_RT);
            mxSetCell(plhs[1],(nWrap*4)*cmpTime+(cmpWrap+2*nWrap),tp_dim1);
            mxSetCell(plhs[1],(nWrap*4)*cmpTime+(cmpWrap+3*nWrap),tp_dim2);

            cmpWrap++;
            if (cmpWrap>=nWrap) {
                cmpWrap = 0;
                cmpTime++;
            }
        }
    }
    return;
}

#endif // BIORBD_MATLAB_MUSCLES_POINTS_H
