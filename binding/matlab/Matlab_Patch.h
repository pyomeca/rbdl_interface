#ifndef BIORBD_MATLAB_PATCH_H
#define BIORBD_MATLAB_PATCH_H

#include <mex.h>
#include "BiorbdModel.h"
#include "class_handle.h"
#include "processArguments.h"
#include "RigidBody/MeshFace.h"

void Matlab_Patch( int, mxArray *plhs[],
                   int nrhs, const mxArray*prhs[] )
{

    // Verifier les arguments d'entree
    checkNombreInputParametres(nrhs, 2, 3,
                               "3 arguments are required (+1 optional) where the 2nd is the handler on the model and the optional 4th is a specific segment index");
    // Recevoir le model
    biorbd::Model * model = convertMat2Ptr<biorbd::Model>(prhs[1]);

    // Recevoir l'index (si envoye)
    int idx(-1);
    if (nrhs==3) {
        idx = getInteger(prhs,2)-1;    // -1 afin que le segment 1 soit le root
    }

    // Output
    if ( idx==-1) { // Si on a demande tous les segments
        // Trouver ou sont les marqueurs
        std::vector<std::vector<biorbd::rigidbody::MeshFace>> allMesh(
                    model->meshFaces());

        // Create a matrix for the return argument
        plhs[0] = mxCreateCellMatrix( allMesh.size(), 1);
        for (unsigned int i_bone=0; i_bone<allMesh.size(); ++i_bone) {
            mxArray *mesh_out_tp = mxCreateDoubleMatrix( 3, allMesh[i_bone].size(), mxREAL);
            double *Mesh = mxGetPr(mesh_out_tp);

            // Remplir le output
            for (unsigned int i=0; allMesh[i_bone].size(); ++i) {
                Mesh[i*3] = allMesh[i_bone][i](0)
                            +1; // +1 Car l'indice dans biorbd::rigidbody::s est par rapport à 0
                Mesh[i*3+1] = allMesh[i_bone][i](1)+1;
                Mesh[i*3+2] = allMesh[i_bone][i](2)+1;
            }
            mxSetCell(plhs[0],i_bone,mesh_out_tp);
        }
        return;

    } else { // Si on a demande un segment precis
        std::vector<biorbd::rigidbody::MeshFace> Mesh_tp(model->meshFaces(
                    static_cast<unsigned int>(idx)));

        // Create a matrix for the return argument
        plhs[0] = mxCreateDoubleMatrix(3, Mesh_tp.size(), mxREAL);
        double *Mesh = mxGetPr(plhs[0]);

        // Remplir le output
        std::vector<biorbd::rigidbody::MeshFace>::iterator it=Mesh_tp.begin();
        for (unsigned int i=0; (it+i)!=Mesh_tp.end(); ++i) {
            Mesh[i*3] = (*(it+i))(0)
                        +1; // +1 Car l'indice dans biorbd::rigidbody::s est par rapport à 0
            Mesh[i*3+1] = (*(it+i))(1)+1;
            Mesh[i*3+2] = (*(it+i))(2)+1;
        }
        return;
    }
}

#endif // BIORBD_MATLAB_PATCH_H
