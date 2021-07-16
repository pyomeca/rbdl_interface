#ifndef BIORBD_MATLAB_MESH_H
#define BIORBD_MATLAB_MESH_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"

void Matlab_Mesh( int, mxArray *plhs[],
                  int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 3, 4,
                               "3 arguments are required (+1 optional) where the 2nd is the handler on the model, 3rd is the Q and 4th optional is a specific segment index");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);
    unsigned int nQ = model->nbQ(); // Get the number of DoF

    // Recevoir Q
    biorbd::rigidbody::GeneralizedCoordinates Q = *getParameterQ(prhs, 2,
            nQ).begin();

    // Recevoir l'index (si envoye)
    int idx(-1);
    if (nrhs==4) {
        idx = getInteger(prhs,3)-1;    // -1 afin que le segment 1 soit le root
    }

    // Output
    if ( idx==-1) { // Si on a demande tous les segments
        // Trouver ou sont les marqueurs
        std::vector<std::vector<biorbd::utils::Vector3d>> allMesh(model->meshPoints(Q));

        // Create a matrix for the return argument
        plhs[0] = mxCreateCellMatrix( allMesh.size(), 1);
        for (unsigned int i_bone=0; i_bone<allMesh.size(); ++i_bone) {
            mxArray *mesh_out_tp = mxCreateDoubleMatrix( 3,
                                   (*(allMesh.begin()+i_bone)).size(), mxREAL);
            double *Mesh = mxGetPr(mesh_out_tp);

            // Remplir le output
            for (unsigned int i=0; i<allMesh[i_bone].size(); ++i) {
                Mesh[i*3] = allMesh[i_bone][i](0);
                Mesh[i*3+1] = allMesh[i_bone][i](1);
                Mesh[i*3+2] = allMesh[i_bone][i](2);
            }
            mxSetCell(plhs[0],i_bone,mesh_out_tp);
        }
        return;

    } else { // Si on a demande un segment precis
        std::vector<biorbd::utils::Vector3d> Mesh_tp(model->meshPoints(Q,
                static_cast<unsigned int>(idx)));

        // Create a matrix for the return argument
        plhs[0] = mxCreateDoubleMatrix(3, Mesh_tp.size(), mxREAL);
        double *Mesh = mxGetPr(plhs[0]);

        // Remplir le output
        for (unsigned int i=0; i<Mesh_tp.size(); ++i) {
            Mesh[i*3] = Mesh_tp[i](0);
            Mesh[i*3+1] = Mesh_tp[i](1);
            Mesh[i*3+2] = Mesh_tp[i](2);
        }
        return;
    }
}

#endif // BIORBD_MATLAB_MESH_H
