#define BIORBD_API_EXPORTS
#include "ModelWriter.h"

#include <iostream>
#include <fstream>

#include "BiorbdModel.h"
#include "Utils/String.h"
#include "Utils/Path.h"
#include "RigidBody/IMU.h"
#include "RigidBody/NodeSegment.h"
#include "RigidBody/Segment.h"
#include "RigidBody/Mesh.h"
#include "RigidBody/SegmentCharacteristics.h"

#ifndef BIORBD_USE_CASADI_MATH
void biorbd::Writer::writeModel(biorbd::Model & model,
                                const biorbd::utils::Path& pathToWrite)
{
    biorbd::utils::String sep("\t"); // separator in the file
    biorbd::utils::String com("//"); // commentaire

    // Manage the case where the destination folder does not exist
    if(!pathToWrite.isFolderExist()) {
        pathToWrite.createFolder();
    }

    //  Open file
    std::ofstream biorbdModelFile;
    biorbdModelFile.open(pathToWrite.relativePath().c_str());

    // Write file
    biorbdModelFile << "version 3" << std::endl;
    biorbdModelFile << std::endl;

    // General information
    biorbdModelFile << std::endl;
    biorbdModelFile << com << " General informations" << std::endl;
    biorbdModelFile << std::endl;

    // Information on the segments
    std::vector<biorbd::utils::RotoTrans> localJCS = model.localJCS();
    for (unsigned int i = 0; i<model.nbSegment(); ++i) {
        biorbdModelFile << com << " Informations about " << model.segment(
                            i).name() << " segment" << std::endl;
        biorbdModelFile << sep << com << " Segment" << std::endl;
        biorbdModelFile << sep << "segment" << sep << model.segment(
                            i).name() << std::endl;
        biorbdModelFile << sep << sep << "parent" << sep << model.segment(
                            i).parent() << std::endl;
        biorbdModelFile << sep << sep << "RTinMatrix" << sep << true << std::endl;
        biorbdModelFile << sep << sep << "RT" << std::endl;
        for (unsigned int j=0; j<4; ++j) {
            biorbdModelFile << sep << sep << sep << localJCS[i].block(j,0,1,4) << std::endl;
        }
        if (model.segment(i).nbDofTrans() > 0) {
            biorbdModelFile << sep << sep << "translations" << sep << model.segment(
                                i).seqT() << std::endl;
        }
        if (model.segment(i).nbDofRot() > 0) {
            biorbdModelFile << sep << sep << "rotations" << sep << model.segment(
                                i).seqR() << std::endl;
        }
        biorbdModelFile << sep << sep << "mass" << sep << model.segment(
                            i).characteristics().mass() << std::endl;
        biorbdModelFile << sep << sep << "inertia" << std::endl << sep << sep << sep <<
                        model.segment(
                            i).characteristics().inertia() << std::endl;
        biorbdModelFile << sep << sep << "com" << sep << model.segment(
                            i).characteristics().mCenterOfMass.transpose() <<
                        std::endl;
        if (model.segment(
                    i).characteristics().mesh().path().filename().compare("")) { // Si ce n'est pas vide
            biorbdModelFile << sep << sep << "meshfile" << sep << model.segment(
                                i).characteristics().mesh().path().originalPath() <<
                            std::endl;
        }
        biorbdModelFile << sep << "endsegment" << sep << std::endl;
        biorbdModelFile << std::endl;

        // Write the prospective markers
        std::vector<biorbd::rigidbody::NodeSegment> markers (model.marker(model.segment(
                    i).name()));
        if (markers.size() > 0) {
            biorbdModelFile << sep << com << " Markers" << std::endl;
            for (size_t j = 0; j< markers.size(); ++j) {
                biorbdModelFile << sep << "marker" << sep <<
                                markers[j].biorbd::utils::Node::name() << std::endl;
                biorbdModelFile << sep << sep << "parent" << sep << markers[j].parent() <<
                                std::endl;
                biorbdModelFile << sep << sep << "position" << sep << markers[j].transpose() <<
                                std::endl;
                biorbdModelFile << sep << sep << "technical" << sep << markers[j].isTechnical()
                                << std::endl;
                biorbdModelFile << sep << sep << "anatomical" << sep <<
                                markers[j].isAnatomical() << std::endl;
                if (markers[j].nbAxesToRemove() != 0) {
                    biorbdModelFile << sep << sep << "axestoremove" << sep <<
                                    markers[j].axesToRemove() << std::endl;
                }
                biorbdModelFile << sep << "endmarker" << sep << std::endl;
            }
        }
        biorbdModelFile << std::endl;

        // Write the inertial units
        std::vector<biorbd::rigidbody::IMU> imus(model.IMU(model.segment(i).name()));
        if (imus.size() > 0) {
            biorbdModelFile << sep << com << " Inertial Magnetic Unit" << std::endl;
            for (size_t j = 0; j< imus.size(); ++j) {
                biorbdModelFile << sep << "imu" << sep << imus[j].biorbd::utils::Node::name() <<
                                std::endl;
                biorbdModelFile << sep << sep << "parent" << sep << imus[j].parent() <<
                                std::endl;
                biorbdModelFile << sep << sep << "RTinMatrix" << sep << true << std::endl;
                biorbdModelFile << sep << sep << "RT" << std::endl;
                for (unsigned int k=0; k<4; ++k) {
                    biorbdModelFile << sep << sep << sep << imus[j].block(k,0,1,4) << std::endl;
                }
                biorbdModelFile << sep << sep << "technical" << sep << imus[j].isTechnical() <<
                                std::endl;
                biorbdModelFile << sep << sep << "anatomical" << sep << imus[j].isAnatomical()
                                << std::endl;
                biorbdModelFile << sep << "endimu" << sep << std::endl;
            }
        }


        // Write the custom RT
        std::vector<biorbd::utils::RotoTransNode> rts(model.RTs(model.segment(
                    i).name()));
        if (rts.size() > 0) {
            biorbdModelFile << sep << com << " Custom RT" << std::endl;
            for (size_t j = 0; j< rts.size(); ++j) {
                biorbdModelFile << sep << "customRT" << sep <<
                                rts[j].biorbd::utils::Node::name() << std::endl;
                biorbdModelFile << sep << sep << "parent" << sep << rts[j].parent() <<
                                std::endl;
                biorbdModelFile << sep << sep << "RTinMatrix" << sep << true << std::endl;
                biorbdModelFile << sep << sep << "RT" << std::endl;
                for (unsigned int k=0; k<4; ++k) {
                    biorbdModelFile << sep << sep << sep << rts[j].block(k,0,1,4) << std::endl;
                }
                biorbdModelFile << sep << "endcustomrt" << sep << std::endl;
            }
        }
        biorbdModelFile << std::endl;

        biorbdModelFile << std::endl;
    }
    biorbdModelFile << std::endl;
    biorbdModelFile << std::endl;
    biorbdModelFile << std::endl;
    biorbdModelFile << std::endl;
    biorbdModelFile << std::endl;
    biorbdModelFile << std::endl;



    biorbdModelFile << std::endl;
    // Close file
    biorbdModelFile.close();

}
#endif
