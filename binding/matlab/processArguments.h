#ifndef MATLAB_PROCESS_ARGUMENTS_H
#define MATLAB_PROCESS_ARGUMENTS_H
#include <mex.h>
#include "Utils/String.h"
#include "Utils/Matrix.h"
#include "Utils/Rotation.h"
#include "Utils/SpatialVector.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/GeneralizedVelocity.h"
#include "RigidBody/GeneralizedAcceleration.h"
#include "RigidBody/GeneralizedTorque.h"
#include "RigidBody/Segment.h"
#include "RigidBody/IMU.h"
#include "RigidBody/NodeSegment.h"
#include "Muscles/StateDynamics.h"
#include "Muscles/StateDynamicsBuchanan.h"

void checkNombreInputParametres(int nrhs, int min, int max,
                                std::string message = "")
{
    if (nrhs < min) {
        mexErrMsgIdAndTxt("MATLAB:yprime:invalidNumInputs",
                          message.c_str());
    } else if (nrhs > max) {
        mexErrMsgIdAndTxt("MATLAB:yprime:invalidNumInputs",
                          message.c_str());
    }
}

std::vector<std::vector<biorbd::rigidbody::NodeSegment>> getParameterAllMarkers(
            const mxArray*prhs[], unsigned int idx,
            int nMark=-1)
{
    // Check data type of input argument
    if (!(mxIsDouble(prhs[idx]))) {
        std::ostringstream msg;
        msg << "Argument " << idx+1 << " must be of type double.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }

    mwSize m=mxGetM(prhs[idx]); // XYZ
    mwSize n=mxGetN(prhs[idx]); // Nombre de markers

    // Get the number of elements in the input argument
    if (m<3 || m>4) {
        std::ostringstream msg;
        msg << "Wrong size! Input markers matrix should be 3 or 4 lines";
        mexErrMsgTxt(msg.str().c_str());
    }

    // Get number of dimension
    mwSize nsubs=mxGetNumberOfDimensions(prhs[idx]);
    if (nsubs < 2 || nsubs > 3) {
        std::ostringstream msg;
        msg << "Wrong size! Input markers matrix should be a 3-dimensions or 2-dimensions matrix";
        mexErrMsgTxt(msg.str().c_str());
    }
    // Get number of timeframes
    double nFrames(1);
    if (nMark==-1) {
        nMark = static_cast<int>((mxGetDimensions(prhs[idx]))[1]);
    }
    if (nsubs == 3) {
        nFrames = (mxGetDimensions(prhs[idx]))[2];
    }

    // Get the number of elements in the input argument
    if (static_cast<int>(n/nFrames) !=
            nMark) { // puisque les dimensions supplémentaires sont ajoutées a la fin, il faut / par nFrames
        std::ostringstream msg;
        msg << "Wrong size! Input markers matrix should should have the same number of technical markers ("
            << nMark <<
            " markers.";
        mexErrMsgTxt(msg.str().c_str());
    }

    // Get a pointer to the values
    double *markers = mxGetPr(prhs[idx]);



    // Créer la sortie
    std::vector<std::vector<biorbd::rigidbody::NodeSegment>> markersOverTime;

    // Stocker les valeurs dans le format de sortie
    unsigned int cmp(0);
    for (unsigned int i=0; i<nFrames; ++i) {
        std::vector<biorbd::rigidbody::NodeSegment> markers_tp; // Markers a un temps i

        for (int j=0; j<nMark; ++j) {
            biorbd::rigidbody::NodeSegment tp(markers[m*cmp+0], // m est 3 ou 4
                                              markers[m*cmp+1],
                                              markers[m*cmp+2]);
            markers_tp.push_back(tp);
            ++cmp;
        }
        markersOverTime.push_back(markers_tp);
    }

    // Retourner la matrice
    return markersOverTime;
}
std::vector<std::vector<biorbd::rigidbody::IMU>> getParameterAllIMUs(
            const mxArray*prhs[], unsigned int idx)
{
    // Check data type of input argument
    if (!(mxIsDouble(prhs[idx]))) {
        std::ostringstream msg;
        msg << "Argument " << idx+1 << " must be of type double.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }

    // Get number of dimension
    mwSize nsubs=mxGetNumberOfDimensions(prhs[idx]);
    mwSize m=(mxGetDimensions(prhs[idx]))[0]; // nligne
    mwSize n=(mxGetDimensions(prhs[idx]))[1]; // ncol

    // Get the number of elements in the input argument
    if (m!=n) {
        std::ostringstream msg;
        msg << "Wrong size! IMUs must be a 3x3 or 4x4 matrix";
        mexErrMsgTxt(msg.str().c_str());
    }
    if (m<3 || m>4) {
        std::ostringstream msg;
        msg << "Wrong size! IMUs must be a 3x3 or 4x4 matrix";
        mexErrMsgTxt(msg.str().c_str());
    }


    if (nsubs < 2 || nsubs > 4) {
        std::ostringstream msg;
        msg << "Wrong size! Input IMUs matrix should be a 3-dimensions or 2-dimensions matrix";
        mexErrMsgTxt(msg.str().c_str());
    }
    // Get number of IMUs
    mwSize nIMUs(1);
    if (nsubs >= 3) {
        nIMUs = (mxGetDimensions(prhs[idx]))[2];
    }

    // Get number of timeframes
    double nFrames(1);
    if (nsubs >= 4) {
        nFrames = (mxGetDimensions(prhs[idx]))[3];
    }

    // Get a pointer to the values
    double *imus = mxGetPr(prhs[idx]);

    // Créer la sortie
    std::vector<std::vector<biorbd::rigidbody::IMU>> imuOverTime;

    // Stocker les valeurs dans le format de sortie
    for (unsigned int i=0; i<nFrames; ++i) {
        std::vector<biorbd::rigidbody::IMU> imus_tp; // IMUs a un temps i
        for (unsigned int j=0; j<nIMUs; ++j) {
            biorbd::utils::Rotation rot;
            biorbd::utils::Vector3d trans;
            if (n==4) {
                rot <<  imus[i*16*nIMUs+j*16+0], imus[i*16*nIMUs+j*16+4],
                    imus[i*16*nIMUs+j*16+8],
                    imus[i*16*nIMUs+j*16+1], imus[i*16*nIMUs+j*16+5], imus[i*16*nIMUs+j*16+9],
                    imus[i*16*nIMUs+j*16+2], imus[i*16*nIMUs+j*16+6], imus[i*16*nIMUs+j*16+10];
                trans << imus[i*16*nIMUs+j*16+12], imus[i*16*nIMUs+j*16+13],
                      imus[i*16*nIMUs+j*16+14];
            } else {
                rot <<  imus[i*9*nIMUs+j*9+0], imus[i*9*nIMUs+j*9+3], imus[i*9*nIMUs+j*9+6],
                    imus[i*9*nIMUs+j*9+1], imus[i*9*nIMUs+j*9+4], imus[i*9*nIMUs+j*9+7],
                    imus[i*9*nIMUs+j*9+2], imus[i*9*nIMUs+j*9+5], imus[i*9*nIMUs+j*9+8];
                trans.setZero();
            }
            imus_tp.push_back(biorbd::utils::RotoTrans(rot, trans));
        }
        imuOverTime.push_back(imus_tp);
    }

    // Retourner la matrice
    return imuOverTime;
}


biorbd::rigidbody::GeneralizedCoordinates getVector(const mxArray*prhs[],
        unsigned int idx, std::string type = "")
{
    // Check data type of input argument
    if (!(mxIsDouble(prhs[idx]))) {
        std::ostringstream msg;
        msg << "Argument " << idx+1 << " must be of type double.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }

    mwSize m=mxGetM(prhs[idx]); // line
    mwSize n=mxGetN(prhs[idx]); // line

    mwSize length(0);
    if (m!=1 && n==
            1) { // on permet le nombre juste ou juste une valeur qui sera dupliquée
        length = m;
    } else if (n!=1 && m == 1) {
        length = n;
    } else {
        std::ostringstream msg;
        msg << type << " should be a vector";
        mexErrMsgTxt(msg.str().c_str());
    }

    double *q=mxGetPr(prhs[idx]); //matrice de position

    // Coordonnées généralisées du modèle envoyées vers lisible par le modèle
    biorbd::rigidbody::GeneralizedCoordinates vect(static_cast<unsigned int>
            (length));
    for (unsigned int i=0; i<length; i++) {
        vect(i) = q[i];
    }

    return vect;
}

biorbd::rigidbody::GeneralizedCoordinates getVector(const mxArray*prhs[],
        unsigned int idx, unsigned int length,
        std::string type = "")
{
    mwSize m=mxGetM(prhs[idx]); // line
    // Get the number of elements in the input argument
    if ( !(m==length
            || m==1) ) { // on permet le nombre juste ou juste une valeur qui sera dupliquée
        std::ostringstream msg;
        msg << "Wrong size! (Input " << type << "), should be " << length;
        mexErrMsgTxt(msg.str().c_str());
    }

    return getVector(prhs, idx, type);
}
Eigen::Vector3d getVector3d(const mxArray*prhs[], unsigned int idx)
{
    return getVector(prhs, idx, 3, "Vector3d");
}

std::vector<biorbd::rigidbody::GeneralizedCoordinates> getParameterQ(
    const mxArray*prhs[], unsigned int idx,
    unsigned int nDof, std::string type = "q")
{

    // Check data type of input argument
    if (!(mxIsDouble(prhs[idx]))) {
        std::ostringstream msg;
        msg << "Argument " << idx+1 << " must be of type double.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }

    mwSize m=mxGetM(prhs[idx]); // line
    // Get the number of elements in the input argument
    if (m!=nDof) {
        std::ostringstream msg;
        msg << "Wrong size! (Input " << type << "), should be " << nDof;
        mexErrMsgTxt(msg.str().c_str());
    }

    // Get the number of frames in the input argument
    mwSize nFrames=mxGetN(prhs[idx]); // line

    double *q=mxGetPr(prhs[idx]); //matrice de position

    // Coordonnées généralisées du modèle envoyées vers lisible par le modèle
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> Q;
    for (unsigned int j=0; j<nFrames; ++j) {
        biorbd::rigidbody::GeneralizedCoordinates Q_tp(nDof);
        for (unsigned int i=0; i<nDof; i++) {
            Q_tp[i] = q[j*nDof+i];
        }

        Q.push_back(Q_tp);
    }

    return Q;
}
std::vector<biorbd::rigidbody::GeneralizedAcceleration> getParameterQddot(
    const mxArray*prhs[], unsigned int idx, unsigned int nbQddot,
    std::string type = "qddot")
{

    // Check data type of input argument
    if (!(mxIsDouble(prhs[idx]))) {
        std::ostringstream msg;
        msg << "Argument " << idx+1 << " must be of type double.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }

    mwSize m=mxGetM(prhs[idx]); // line
    // Get the number of elements in the input argument
    if (m!=nbQddot) {
        std::ostringstream msg;
        msg << "Wrong size! (Input " << type << "), should be " << nbQddot;
        mexErrMsgTxt(msg.str().c_str());
    }

    // Get the number of frames in the input argument
    mwSize nFrames=mxGetN(prhs[idx]); // line

    double *q=mxGetPr(prhs[idx]); //matrice de position

    // Coordonnées généralisées du modèle envoyées vers lisible par le modèle
    std::vector<biorbd::rigidbody::GeneralizedAcceleration> Qddot;
    for (unsigned int j=0; j<nFrames; ++j) {
        biorbd::rigidbody::GeneralizedAcceleration Qddot_tp(nbQddot);
        for (unsigned int i=0; i<nbQddot; i++) {
            Qddot_tp[i] = q[j*nbQddot+i];
        }

        Qddot.push_back(Qddot_tp);
    }

    return Qddot;
}
std::vector<biorbd::rigidbody::GeneralizedVelocity> getParameterQdot(
    const mxArray*prhs[], unsigned int idx, unsigned int nbQdot,
    std::string type = "qdot")
{

    // Check data type of input argument
    if (!(mxIsDouble(prhs[idx]))) {
        std::ostringstream msg;
        msg << "Argument " << idx+1 << " must be of type double.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }

    mwSize m=mxGetM(prhs[idx]); // line
    // Get the number of elements in the input argument
    if (m!=nbQdot) {
        std::ostringstream msg;
        msg << "Wrong size! (Input " << type << "), should be " << nbQdot;
        mexErrMsgTxt(msg.str().c_str());
    }

    // Get the number of frames in the input argument
    mwSize nFrames=mxGetN(prhs[idx]); // line

    double *q=mxGetPr(prhs[idx]); //matrice de position

    // Coordonnées généralisées du modèle envoyées vers lisible par le modèle
    std::vector<biorbd::rigidbody::GeneralizedVelocity> Qdot;
    for (unsigned int j=0; j<nFrames; ++j) {
        biorbd::rigidbody::GeneralizedVelocity Qdot_tp(nbQdot);
        for (unsigned int i=0; i<nbQdot; i++) {
            Qdot_tp[i] = q[j*nbQdot+i];
        }

        Qdot.push_back(Qdot_tp);
    }

    return Qdot;
}
std::vector<biorbd::rigidbody::GeneralizedTorque> getParameterGeneralizedTorque(
    const mxArray*prhs[], unsigned int idx,
    unsigned int nControl)
{
    std::vector<biorbd::rigidbody::GeneralizedCoordinates> AllGeneralizedTorque_tp =
        getParameterQ(prhs, idx, nControl,
                      "GeneralizedTorque");
    std::vector<biorbd::rigidbody::GeneralizedTorque> AllGeneralizedTorque;

    for (unsigned int j=0; j<AllGeneralizedTorque_tp.size(); ++j) {
        biorbd::rigidbody::GeneralizedTorque GeneralizedTorque_tp(nControl);

        for (unsigned int i=0; i < nControl; ++i) { // Everything else
            GeneralizedTorque_tp(i) = (*(AllGeneralizedTorque_tp.begin()+j))[i];
        }

        AllGeneralizedTorque.push_back(GeneralizedTorque_tp);
    }
    return AllGeneralizedTorque;
}

std::vector<std::vector<biorbd::utils::SpatialVector>> getForcePlate(
            const mxArray*prhs[], unsigned int idx)
{
    if (!(mxIsDouble(prhs[idx]))) {
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",
                           "Argument 6 must be of type double.");
    }

    const mwSize* dims (mxGetDimensions(prhs[idx]));
    mwSize mPF(dims[0]); // Nombre de lignes (devrait etre 6)
    mwSize nPF(dims[1]); // Nombre de plateformes
    mwSize timeStamp;

    if (mPF*nPF == mxGetNumberOfElements(
                prhs[idx]) ) { // s'il n'y a pas de dimension temps
        timeStamp = 1;
    } else {
        timeStamp = dims[2];    // Nombre de temps
    }


    if (mPF!=6) { // must be 6 lines (mx, my, mz, fx, fy, fz)
        std::string errorMessage =
            "Wrong size! (Input forceplates), should be 6xNb_Forceplates x time";
        mexErrMsgTxt(errorMessage.c_str());
    }
    double *pf = mxGetPr(prhs[idx]); // Matrice des plateforme de force

    // stockage des plateformes
    std::vector<std::vector<biorbd::utils::SpatialVector>> PF;
    unsigned int cmp(0);
    for (unsigned int j=0; j<timeStamp; ++j) {
        std::vector<biorbd::utils::SpatialVector> PF_tp;
        for (unsigned int i=0; i<nPF; ++i) { // pour chaque plateforme
            biorbd::utils::SpatialVector tp(pf[0+cmp*6], pf[1+cmp*6], pf[2+cmp*6],
                                            pf[3+cmp*6], pf[4+cmp*6], pf[5+cmp*6]);
            PF_tp.push_back(tp);
            cmp++;
        }
        PF.push_back(PF_tp);
    }

    return PF;
}
double getDouble(const mxArray*prhs[], unsigned int idx, std::string type = "d")
{

    // Check data type of input argument
    if (!(mxIsDouble(prhs[idx]))) {
        std::ostringstream msg;
        msg << "Argument " << idx+1 << " must be of type double.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }

    mwSize m=mxGetM(prhs[idx]); // line
    // Get the number of elements in the input argument
    if (m!=1) {
        std::ostringstream msg;
        msg << "Wrong size! (Input " << type << "), should be 1";
        mexErrMsgTxt(msg.str().c_str());
    }
    double *q=mxGetPr(prhs[idx]); //matrice de position
    return q[0];
}
int getInteger(const mxArray*prhs[], unsigned int idx, std::string type = "i")
{
    return static_cast<int>(getDouble(prhs, idx, type));
}
std::vector<double> getDoubleArray(const mxArray*prhs[], unsigned int idx)
{

    // Check data type of input argument
    if (!(mxIsDouble(prhs[idx]))) {
        std::ostringstream msg;
        msg << "Argument " << idx+1 << " must be of type double.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }

    mwSize m=mxGetM(prhs[idx]); // line
    mwSize n=mxGetN(prhs[idx]); // line
    // Get the number of elements in the input argument
    if (m!=1 && n!=1) {
        std::ostringstream msg;
        msg << "Wrong size! (Input double), should be a vector";
        mexErrMsgTxt(msg.str().c_str());
    }
    double *q=mxGetPr(prhs[idx]); //matrice de position

    std::vector<double> out;
    for (unsigned int i =0; i<m*n; ++i) {
        out.push_back(q[i]);
    }

    return out;
}

biorbd::utils::String getString(const mxArray*prhs[], unsigned int idx)
{
    // Check data type of input argument
    if (!(mxIsChar(prhs[idx]))) {
        std::ostringstream msg;
        msg << "Argument " << idx+1 << " must be of type char.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }

    return mxArrayToString(prhs[idx]);

}

bool getBool(const mxArray*prhs[], unsigned int idx)
{
    bool out(false);

    if (mxIsDouble(prhs[idx])) {
        out = static_cast<bool>(*(mxGetPr(prhs[idx])));
    } else if (mxIsLogical(prhs[idx])) {
        out = *(mxGetLogicals(prhs[idx]));
    } else {
        std::ostringstream msg;
        msg << "Argument " << idx+1 << " must be of type bool.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }
    return out;

}


bool isStateExist(const mxArray*prhs[], unsigned int nMus, int idx,
                  bool &isThere)
{
    if (idx != -1) {
        if (!(mxIsDouble(prhs[idx]))) {
            std::ostringstream msg;
            msg << "Argument " << idx+1 << " must be of type double.";
            mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
        }

        mwSize m=mxGetM(prhs[idx]); // line
        // Get the number of elements in the input argument
        if (m!=nMus) {
            std::ostringstream msg;
            msg << "Wrong size! (Input state muscle parameter), should be " << nMus;
            mexErrMsgTxt(msg.str().c_str());
        }

        isThere = true;
    } else {
        isThere = false;
    }

    return isThere;
}

std::vector<std::vector<std::shared_ptr<biorbd::muscles::State>>>
getParameterMuscleState(
    const mxArray*prhs[],
    int idxExcitation,
    int idxActivation,
    unsigned int nMus)
{
    // Get the number of frames in the input argument
    // Regarde ce qui se passe pour l'excitation
    bool isThereExcitation(false);
    mwSize nFramesE(0);
    double *stateExcitation = nullptr;
    if (isStateExist(prhs, nMus, idxExcitation, isThereExcitation)) {
        stateExcitation=mxGetPr(prhs[idxExcitation]); //matrice de position
        nFramesE = mxGetN(prhs[idxExcitation]); // line
    }

    // Regarde ce qui se passe pour l'activation
    bool isThereActivation(false);
    mwSize nFramesA(0);
    double *stateActivation = nullptr; //matrice de position
    if (isStateExist(prhs, nMus, idxActivation, isThereActivation)) {
        stateActivation=mxGetPr(prhs[idxActivation]); //matrice de position
        nFramesA = mxGetN(prhs[idxActivation]); // line
    }

    // S'assurer que l'excitation et l'activation ont la même taille
    if (isThereActivation && isThereExcitation
            && nFramesA != nFramesE) { // Si on se soucis de l'excitation
        std::ostringstream msg;
        msg << "Wrong size! Input excitation and activation should be the same";
        mexErrMsgTxt(msg.str().c_str());
    }
    mwSize nFramesTotal(static_cast<mwSize>(-1));
    if (isThereActivation) {
        nFramesTotal = nFramesA;
    } else if (isThereExcitation) {
        nFramesTotal = nFramesE;
    } else {
        std::ostringstream msg;
        msg << "At least excitation or activation should be send";
        mexErrMsgTxt(msg.str().c_str());
    }


    // Coordonnées généralisées du modèle envoyées vers lisible par le modèle
    std::vector<std::vector<std::shared_ptr<biorbd::muscles::State>>> States;
    for (unsigned int j=0; j<nFramesTotal; ++j) {
        std::vector<std::shared_ptr<biorbd::muscles::State>> States_tp;
        for (unsigned int i=0; i<nMus; i++)
            if (isThereExcitation && isThereActivation) {
                States_tp.push_back( std::make_shared<biorbd::muscles::State>
                                     (stateExcitation[j*nMus+i], stateActivation[j*nMus+i]));
            } else if (!isThereExcitation && isThereActivation) {
                States_tp.push_back( std::make_shared<biorbd::muscles::State>(0,
                                     stateActivation[j*nMus+i]));
            } else if (isThereExcitation && !isThereActivation) {
                States_tp.push_back( std::make_shared<biorbd::muscles::State>
                                     (stateExcitation[j*nMus+i], 0));
            }

        States.push_back(States_tp);
    }
    return States;
}


std::vector<std::vector<biorbd::muscles::StateDynamicsBuchanan>>
        getParameterMuscleStateBuchanan(const mxArray*prhs[],
                                        int idxExcitation, int idxActivation, unsigned int nMus)
{
    // Get the number of frames in the input argument
    // Regarde ce qui se passe pour l'excitation
    bool isThereExcitation(false);
    mwSize nFramesE(0);
    double *stateExcitation = nullptr;
    if (isStateExist(prhs, nMus, idxExcitation, isThereExcitation)) {
        stateExcitation=mxGetPr(prhs[idxExcitation]); //matrice de position
        nFramesE = mxGetN(prhs[idxExcitation]); // line
    }

    // Regarde ce qui se passe pour l'activation
    bool isThereActivation(false);
    mwSize nFramesA(0);
    double *stateActivation = nullptr; //matrice de position
    if (isStateExist(prhs, nMus, idxActivation, isThereActivation)) {
        stateActivation=mxGetPr(prhs[idxActivation]); //matrice de position
        nFramesA = mxGetN(prhs[idxActivation]); // line
    }

    // S'assurer que l'excitation et l'activation ont la même taille
    if (isThereActivation && isThereExcitation
            && nFramesA != nFramesE) { // Si on se soucis de l'excitation
        std::ostringstream msg;
        msg << "Wrong size! Input excitation and activation should be the same";
        mexErrMsgTxt(msg.str().c_str());
    }
    mwSize nFramesTotal(static_cast<mwSize>(-1));
    if (isThereActivation) {
        nFramesTotal = nFramesA;
    } else if (isThereExcitation) {
        nFramesTotal = nFramesE;
    } else {
        std::ostringstream msg;
        msg << "At least excitation or activation should be send";
        mexErrMsgTxt(msg.str().c_str());
    }


    // Coordonnées généralisées du modèle envoyées vers lisible par le modèle
    std::vector<std::vector<biorbd::muscles::StateDynamicsBuchanan>> States;
    for (unsigned int j=0; j<nFramesTotal; ++j) {
        std::vector<biorbd::muscles::StateDynamicsBuchanan> States_tp;
        for (unsigned int i=0; i<nMus; i++)
            if (isThereExcitation && isThereActivation) {
                States_tp.push_back( biorbd::muscles::StateDynamicsBuchanan(
                                         stateExcitation[j*nMus+i], stateActivation[j*nMus+i]));
            } else if (!isThereExcitation && isThereActivation) {
                States_tp.push_back( biorbd::muscles::StateDynamicsBuchanan(0,
                                     stateActivation[j*nMus+i]));
            } else if (isThereExcitation && !isThereActivation) {
                States_tp.push_back( biorbd::muscles::StateDynamicsBuchanan(
                                         stateExcitation[j*nMus+i], 0));
            }

        States.push_back(States_tp);
    }
    return States;
}

std::vector<std::vector<std::shared_ptr<biorbd::muscles::State>>>
getParameterMuscleStateActivation(
    const mxArray*prhs[], int idxActivation, unsigned int nMus)
{
    return getParameterMuscleState(prhs, -1, idxActivation, nMus);
}
std::vector<std::vector<std::shared_ptr<biorbd::muscles::State>>>
getParameterMuscleStateExcitation(
    const mxArray*prhs[], int idxExcitation, unsigned int nMus)
{
    return getParameterMuscleState(prhs, idxExcitation, -1, nMus);
}

std::vector<std::vector<biorbd::muscles::StateDynamicsBuchanan>>
        getParameterMuscleStateActivationBuchanan(
            const mxArray*prhs[], int idxActivation, unsigned int nMus)
{
    return getParameterMuscleStateBuchanan(prhs, -1, idxActivation, nMus);
}
std::vector<std::vector<biorbd::muscles::StateDynamicsBuchanan>>
        getParameterMuscleStateExcitationBuchanan(
            const mxArray*prhs[], int idxExcitation, unsigned int nMus)
{
    return getParameterMuscleStateBuchanan(prhs, idxExcitation, -1, nMus);
}

std::vector<Eigen::VectorXd> getParameterMuscleForceNorm(const mxArray*prhs[],
        unsigned int idx, unsigned int nMus,
        std::string type = "muscle force")
{
    // Check data type of input argument
    if (!(mxIsDouble(prhs[idx]))) {
        std::ostringstream msg;
        msg << "Argument " << idx+1 << " must be of type double.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }

    mwSize m=mxGetM(prhs[idx]); // line
    // Get the number of elements in the input argument
    if (m!=nMus) {
        std::ostringstream msg;
        msg << "Wrong size! (Input " << type << "), should be " << nMus;
        mexErrMsgTxt(msg.str().c_str());
    }

    // Get the number of frames in the input argument
    mwSize nFrames=mxGetN(prhs[idx]); // line

    double *f=mxGetPr(prhs[idx]); //matrice de position

    // Forces musculaires
    std::vector<Eigen::VectorXd> F;
    for (unsigned int j=0; j<nFrames; ++j) {
        Eigen::VectorXd F_tp = Eigen::VectorXd::Zero (nMus);
        for (unsigned int i=0; i<nMus; i++) {
            F_tp[i] = f[j*nMus+i];
        }

        F.push_back(F_tp);
    }
    return F;
}


std::vector<std::vector<biorbd::utils::Vector3d>> getMusclePosition(
            const mxArray*prhs[], unsigned int idx,
            Eigen::VectorXd nPointsByMuscles)
{
    // Check data type of input argument
    if (!(mxIsDouble(prhs[idx]))) {
        std::ostringstream msg;
        msg << "Argument " << idx+1 << " must be of type double.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }

    mwSize nRows=mxGetM(prhs[idx]); // line
    if (nRows<3 || nRows>4) {
        std::ostringstream msg;
        msg << "Muscle position should be a column-wise matrix with 3 or 4 rows (where the 4th is a line of 1)";
        mexErrMsgTxt(msg.str().c_str());
    }

    // Get the number of muscles in the input argument
    unsigned int nVia(0);
    for (int i = 0; i<nPointsByMuscles.size(); ++i) {
        nVia += nPointsByMuscles(i);
    }

    mwSize nCol=mxGetN(prhs[idx]); // col
    if (nCol!=
            nVia) { // on permet le nombre juste ou juste une valeur qui sera dupliquée
        std::ostringstream msg;
        msg << "muscle position should be a column-wise matrix with columns = " << nVia;
        mexErrMsgTxt(msg.str().c_str());
    }

    // Recueillir les données
    double *via=mxGetPr(prhs[idx]); //matrice de position

    // Préparer la matrice de sortie
    std::vector<std::vector<biorbd::utils::Vector3d>> out;
    unsigned int cmpMus(0);
    for (unsigned int i=0; i<nPointsByMuscles.rows(); ++i) {
        // Préparer les matrices intermédiaires (chaque muscle)
        std::vector<biorbd::utils::Vector3d> mus;
        for (unsigned int j=0; j<nPointsByMuscles(i); ++j) {
            mus.push_back(biorbd::utils::Vector3d(via[cmpMus*nRows+0],via[cmpMus*nRows+1],
                                                  via[cmpMus*nRows+2]));
            ++cmpMus;
        }
        out.push_back(mus);
    }

    // Retourner la structure remplie
    return out;
}

std::vector<biorbd::utils::Matrix> getMusclePointsJaco(const mxArray*prhs[],
        unsigned int idx,
        Eigen::VectorXd nPointsByMuscles, unsigned int nQ)
{
    // Check data type of input argument
    if (!(mxIsDouble(prhs[idx]))) {
        std::ostringstream msg;
        msg << "Argument " << idx+1 << " must be of type double.";
        mexErrMsgIdAndTxt( "MATLAB:findnz:invalidInputType",msg.str().c_str());
    }

    mwSize nRows=mxGetM(prhs[idx]); // line

    unsigned int nVia(0);
    for (int i = 0; i<nPointsByMuscles.size(); ++i) {
        nVia += static_cast<unsigned int>(nPointsByMuscles(i));
    }
    if (nRows!=nVia*3) {
        std::ostringstream msg;
        msg << "Jacobian must be a matrix with 3*nMus*nVia/nMus X nQ (" << nVia*3 << ","
            << nQ << ")";
        mexErrMsgTxt(msg.str().c_str());
    }

    mwSize nCol=mxGetN(prhs[idx]); // col
    if (nCol!=nQ) {
        std::ostringstream msg;
        msg << "Jacobian must be a matrix with 3*nMus X nQ";
        mexErrMsgTxt(msg.str().c_str());
    }

    // Recueillir les données
    double *jaco=mxGetPr(prhs[idx]); //matrice de position

    // Préparer la matrice de sortie
    std::vector<biorbd::utils::Matrix> jacoOut;
    unsigned int cmpMus(0);

    for (unsigned int i=0; i<nPointsByMuscles.rows(); ++i) {
        // Préparer les matrices intermédiaires (chaque muscle)
        biorbd::utils::Matrix mus(static_cast<unsigned int>(nPointsByMuscles(i))*3, nQ);
        for (unsigned int j=0; j<static_cast<unsigned int>(nPointsByMuscles(i)); ++j) {
            // Stocker
            for (unsigned int k1 = 0; k1<3; ++k1)
                for (unsigned int k2 = 0; k2<nQ; ++k2) {
                    mus(3*j+k1, k2) = jaco[3*cmpMus+k1+nVia*3*k2];
                }
            // Incrémenter l'index de muscle
            ++cmpMus;
        }

        jacoOut.push_back(mus);
    }

    // Retourner la structure remplie
    return jacoOut;

}
#endif
