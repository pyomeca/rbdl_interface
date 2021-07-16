#define BIORBD_API_EXPORTS
#include "ModelReader.h"

#include <limits.h>

#include "BiorbdModel.h"
#include "Utils/Error.h"
#include "Utils/IfStream.h"
#include "Utils/String.h"
#include "Utils/Equation.h"
#include "Utils/Vector.h"
#include "Utils/Vector3d.h"
#include "Utils/Rotation.h"
#include "Utils/Range.h"
#include "Utils/SpatialVector.h"
#include "RigidBody/GeneralizedCoordinates.h"
#include "RigidBody/Mesh.h"
#include "RigidBody/SegmentCharacteristics.h"
#include "RigidBody/IMU.h"
#include "RigidBody/MeshFace.h"
#include "RigidBody/NodeSegment.h"

#ifdef MODULE_ACTUATORS
    #include "Actuators/ActuatorConstant.h"
    #include "Actuators/ActuatorLinear.h"
    #include "Actuators/ActuatorGauss3p.h"
    #include "Actuators/ActuatorGauss6p.h"
    #include "Actuators/ActuatorSigmoidGauss3p.h"
#endif // MODULE_ACTUATORS

#ifdef MODULE_MUSCLES
    #include "Muscles/Muscle.h"
    #include "Muscles/Geometry.h"
    #include "Muscles/MuscleGroup.h"
    #include "Muscles/ViaPoint.h"
    #include "Muscles/WrappingHalfCylinder.h"
    #include "Muscles/FatigueParameters.h"
    #include "Muscles/State.h"
    #include "Muscles/Characteristics.h"
    #include "Muscles/ViaPoint.h"
    #include "Muscles/PathModifiers.h"
    #include "Muscles/StateDynamicsBuchanan.h"
#endif // MODULE_MUSCLES

// ------ Public methods ------ //
biorbd::Model biorbd::Reader::readModelFile(const biorbd::utils::Path &path)
{
    // Add the elements that have been entered
    biorbd::Model model;
    readModelFile(path, &model);
    return model;
}

void biorbd::Reader::readModelFile(
    const biorbd::utils::Path &path,
    biorbd::Model *model)
{
    // Open file
    if (!path.isFileReadable())
        biorbd::utils::Error::raise("File " + path.absolutePath()
                                    + " could not be open");

#ifdef _WIN32
    biorbd::utils::IfStream file(
        biorbd::utils::Path::toWindowsFormat(
            path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
        path.absolutePath().c_str(), std::ios::in);
#endif

    // Read file
    biorbd::utils::String main_tag;
    biorbd::utils::String property_tag;
    biorbd::utils::String subproperty_tag;

    // Variable used to replace doubles
    std::map<biorbd::utils::Equation, double> variable;

    // Determine the file version
    file.readSpecificTag("version", main_tag);
    unsigned int version(static_cast<unsigned int>(atoi(main_tag.c_str())));
    biorbd::utils::Error::check((version == 1 || version == 2 || version == 3
                                 || version == 4),
                                "Version " + main_tag + " is not implemented yet");

#ifdef MODULE_ACTUATORS
    bool hasActuators = false;
#endif // MODULE_ACTUATORS

    biorbd::utils::String name;
    try {
        while(file.read(
                    main_tag)) { // Attempt read into main_tag, return false if it fails
            // Reinitialize some tags
            name = "";
            property_tag = "";
            subproperty_tag = "";

            // If it is a segment
            if (!main_tag.tolower().compare("segment")) {
                file.read(name);
                biorbd::utils::String parent_str("root");
                biorbd::utils::String trans = "";
                biorbd::utils::String rot = "";
                bool RTinMatrix(true);
                if (version >= 3) { // By default for version 3 (not in matrix)
                    RTinMatrix = false;
                }
                bool isRTset(false);
                double mass = 0.00000001;
                RigidBodyDynamics::Math::Matrix3d inertia(
                    RigidBodyDynamics::Math::Matrix3d::Identity());
                biorbd::utils::RotoTrans RT(RigidBodyDynamics::Math::Matrix4d::Identity());
                biorbd::utils::Vector3d com(0,0,0);
                biorbd::rigidbody::Mesh mesh;
                int segmentByFile(-1); // -1 non setté, 0 pas par file, 1 par file
                int PF = -1;
                std::vector<biorbd::utils::Range> QRanges;
                std::vector<biorbd::utils::Range> QDotRanges;
                std::vector<biorbd::utils::Range> QDDotRanges;
                bool isRangeQSet(
                    false); // Ranges must be done only after translation AND rotations tags
                bool isRangeQDotSet(
                    false); // Ranges must be done only after translation AND rotations tags
                bool isRangeQDDotSet(
                    false); // Ranges must be done only after translation AND rotations tags
                while(file.read(property_tag) && property_tag.tolower().compare("endsegment")) {
                    if (!property_tag.tolower().compare("parent")) {
                        // Dynamically find the parent number
                        file.read(parent_str);
                        if (parent_str.tolower().compare("root")) {
                            biorbd::utils::Error::check(model->GetBodyId(parent_str.c_str()),
                                                        "Wrong name in a segment");
                        }
                    } else if (!property_tag.tolower().compare("translations")) {
                        biorbd::utils::Error::check(!isRangeQSet,
                                                    "Translations must appear before the rangesq tag");
                        biorbd::utils::Error::check(!isRangeQDotSet,
                                                    "Translations must appear before the rangesqdot tag");
                        biorbd::utils::Error::check(!isRangeQDDotSet,
                                                    "Translations must appear before the rangesqddot tag");
                        file.read(trans);
                    } else if (!property_tag.tolower().compare("rotations")) {
                        biorbd::utils::Error::check(!isRangeQSet,
                                                    "Rotations must appear before the rangesq tag");
                        biorbd::utils::Error::check(!isRangeQDotSet,
                                                    "Rotations must appear before the rangesqdot tag");
                        biorbd::utils::Error::check(!isRangeQDDotSet,
                                                    "Rotations must appear before the rangesqddot tag");
                        file.read(rot);
                    } else if (!property_tag.tolower().compare("ranges") ||
                               !property_tag.tolower().compare("rangesq")) {
                        double min, max;
                        size_t rotLength(0);
                        if (rot.compare("q")) {
                            // If not a quaternion
                            rotLength = rot.length();
                        } else {
                            rotLength = 4;
                        }
                        for (size_t i=0; i<trans.length() + rotLength; ++i) {
                            file.read(min);
                            file.read(max);
                            QRanges.push_back(
                                biorbd::utils::Range (min, max));
                        }
                        isRangeQSet = true;
                    } else if (!property_tag.tolower().compare("rangesqdot")) {
                        double min, max;
                        size_t rotLength(0);
                        if (rot.compare("q")) {
                            // If not a quaternion
                            rotLength = rot.length();
                        } else {
                            rotLength = 3;
                        }
                        for (size_t i=0; i<trans.length() + rotLength; ++i) {
                            file.read(min);
                            file.read(max);
                            QDotRanges.push_back(
                                biorbd::utils::Range (min, max));
                        }
                        isRangeQDotSet = true;
                    } else if (!property_tag.tolower().compare("rangesqddot")) {
                        double min, max;
                        size_t rotLength(0);
                        if (rot.compare("q")) {
                            // If not a quaternion
                            rotLength = rot.length();
                        } else {
                            rotLength = 3;
                        }
                        for (size_t i=0; i<trans.length() + rotLength; ++i) {
                            file.read(min);
                            file.read(max);
                            QDDotRanges.push_back(
                                biorbd::utils::Range (min, max));
                        }
                        isRangeQDDotSet = true;
                    } else if (!property_tag.tolower().compare("mass")) {
                        file.read(mass, variable);
                    } else if (!property_tag.tolower().compare("inertia")) {
                        readMatrix33(file, variable, inertia);
                    } else if (!property_tag.tolower().compare("rtinmatrix")) {
                        biorbd::utils::Error::check(isRTset==false,
                                                    "RT should not appear before RTinMatrix");
                        file.read(RTinMatrix);
                    } else if (!property_tag.tolower().compare("rt")) {
                        readRtMatrix(file, variable, RTinMatrix, RT);
                        isRTset = true;
                    } else if (!property_tag.tolower().compare("com")) {
                        readVector3d(file, variable, com);
                    } else if (!property_tag.tolower().compare("forceplate")
                               || !property_tag.tolower().compare("externalforceindex")) {
                        file.read(PF);
                    } else if (!property_tag.tolower().compare("mesh")) {
                        if (segmentByFile==-1) {
                            segmentByFile = 0;
                        } else if (segmentByFile == 1) {
                            biorbd::utils::Error::raise("You must not mix file and mesh in segment");
                        }
                        biorbd::utils::Vector3d tp(0, 0, 0);
                        readVector3d(file, variable, tp);
                        mesh.addPoint(tp);
                    } else if (!property_tag.tolower().compare("patch")) {
                        if (segmentByFile==-1) {
                            segmentByFile = 0;
                        } else if (segmentByFile == 1) {
                            biorbd::utils::Error::raise("You must not mix file and mesh in segment");
                        }
                        biorbd::rigidbody::MeshFace tp;
                        for (unsigned int i=0; i<3; ++i) {
                            file.read(tp(i));
                        }
                        mesh.addFace(tp);
                    } else if (!property_tag.tolower().compare("meshfile")) {
                        if (segmentByFile==-1) {
                            segmentByFile = 1;
                        } else if (segmentByFile == 0) {
                            biorbd::utils::Error::raise("You must not mix file and mesh in segment");
                        }
                        biorbd::utils::String filePathInString;
                        file.read(filePathInString);
                        biorbd::utils::Path filePath(filePathInString);
                        if (!filePath.extension().compare("bioMesh")) {
                            mesh = readMeshFileBiorbdSegments(path.folder() + filePath.relativePath());
                        } else if (!filePath.extension().compare("ply")) {
                            mesh = readMeshFilePly(path.folder() + filePath.relativePath());
                        } else if (!filePath.extension().compare("obj")) {
                            mesh = readMeshFileObj(path.folder() + filePath.relativePath());
                        }
#ifdef MODULE_VTP_FILES_READER
                        else if (!filePath.extension().compare("vtp")) {
                            mesh = readMeshFileVtp(path.folder() + filePath.relativePath());
                        }
#endif
                        else {
                            biorbd::utils::Error::raise(filePath.extension() +
                                                        " is an unrecognized mesh file");
                        }
                    }
                }
                if (!isRangeQSet) {
                    size_t rotLength(0);
                    if (rot.compare("q")) {
                        // If not a quaternion
                        rotLength = rot.length();
                    } else {
                        rotLength = 4;
                    }
                    for (size_t i=0; i<trans.length() + rotLength; ++i) {
                        if (!rot.compare("q") && i>=trans.length()) {
                            QRanges.push_back(
                                biorbd::utils::Range (-1, 1));
                        } else {
                            QRanges.push_back(
                                biorbd::utils::Range ());
                        }
                    }
                }
                if (!isRangeQDotSet) {
                    size_t rotLength(0);
                    if (rot.compare("q")) {
                        // If not a quaternion
                        rotLength = rot.length();
                    } else {
                        rotLength = 3;
                    }
                    for (size_t i=0; i<trans.length() + rotLength; ++i) {
                        QDotRanges.push_back(
                            biorbd::utils::Range (-M_PI*10, M_PI*10));
                    }
                }
                if (!isRangeQDDotSet) {
                    size_t rotLength(0);
                    if (rot.compare("q")) {
                        // If not a quaternion
                        rotLength = rot.length();
                    } else {
                        rotLength = 3;
                    }
                    for (size_t i=0; i<trans.length() + rotLength; ++i) {
                        QDDotRanges.push_back(
                            biorbd::utils::Range (-M_PI*100, M_PI*100));
                    }
                }
                biorbd::rigidbody::SegmentCharacteristics characteristics(mass,com,inertia,
                        mesh);
                model->AddSegment(name, parent_str, trans, rot, QRanges, QDotRanges,
                                  QDDotRanges, characteristics, RT, PF);
            } else if (!main_tag.tolower().compare("gravity")) {
                biorbd::utils::Vector3d gravity(0,0,0);
                readVector3d(file, variable, gravity);
                model->gravity = gravity;
            } else if (!main_tag.tolower().compare("variables")) {
                biorbd::utils::String var;
                while(file.read(var) && var.tolower().compare("endvariables")) {
                    if (!var(0).compare("$")) {
                        double value;
                        file.read(value);
                        biorbd::utils::Error::check(variable.find(var) == variable.end(),
                                                    "Variable already defined");
                        variable[var] = value;
                    }
                }
            } else if (!main_tag.tolower().compare("marker")) {
                biorbd::utils::String name;
                file.read(name);
                unsigned int parent_int = 0;
                biorbd::utils::String parent_str("root");
                biorbd::utils::Vector3d pos(0,0,0);
                bool technical = true;
                bool anatomical = false;
                biorbd::utils::String axesToRemove;
                while(file.read(property_tag) && property_tag.tolower().compare("endmarker"))
                    if (!property_tag.tolower().compare("parent")) {
                        // Dynamically find the parent number
                        file.read(parent_str);
                        parent_int = model->GetBodyId(parent_str.c_str());
                        // if parent_int still equals zero, no name has concurred
                        biorbd::utils::Error::check(model->IsBodyId(parent_int),
                                                    "Wrong name in a segment");
                    } else if (!property_tag.tolower().compare("position")) {
                        readVector3d(file, variable, pos);
                    } else if (!property_tag.tolower().compare("technical")) {
                        file.read(technical);
                    } else if (!property_tag.tolower().compare("anatomical")) {
                        file.read(anatomical);
                    } else if (!property_tag.tolower().compare("axestoremove")) {
                        file.read(axesToRemove);
                    }

                model->addMarker(pos, name, parent_str, technical, anatomical, axesToRemove,
                                 static_cast<int>(parent_int));
            } else if (!main_tag.tolower().compare("mimu") && version >= 4) {
                biorbd::utils::Error::raise("MIMU is no more the right tag, change it to IMU!");
            } else if (!main_tag.tolower().compare("imu")
                       || !main_tag.tolower().compare("mimu")
                       || !main_tag.tolower().compare("customrt")) {
                biorbd::utils::String rtType(main_tag.tolower());
                biorbd::utils::String name;
                file.read(name);
                biorbd::utils::String parent_str("root");
                biorbd::utils::RotoTransNode RT;
                bool RTinMatrix(true);
                if (version >= 3) { // By default for version 3 (not in matrix)
                    RTinMatrix = false;
                }
                bool isRTset(false);
                bool technical = true;
                bool anatomical = false;
                bool firstTag = true;
                bool fromMarkers(false);
                biorbd::utils::String originMarkerName("");
                biorbd::utils::String firstAxis("");
                std::vector<biorbd::utils::String> firstAxisMarkerNames(2);
                biorbd::utils::String secondAxis("");
                std::vector<biorbd::utils::String> secondAxisMarkerNames(2);
                biorbd::utils::String axisToRecalculate("");
                while(file.read(property_tag) && !(!property_tag.tolower().compare("endimu")
                                                   || !property_tag.tolower().compare("endmimu")
                                                   || !property_tag.tolower().compare("endcustomrt"))) {
                    if (!property_tag.tolower().compare("parent")) {
                        // Dynamically find the parent number
                        file.read(parent_str);
                        // If parent_int still equals zero, no name has concurred
                        biorbd::utils::Error::check(model->IsBodyId(model->GetBodyId(
                                                        parent_str.c_str())), "Wrong name in a segment");
                    } else if (!property_tag.tolower().compare("frommarkers")) {
                        if (!firstTag) {
                            biorbd::utils::Error::raise("The tag 'fromMarkers' should appear first in the IMU "
                                                        + name);
                        } else {
                            fromMarkers = true;
                        }
                    } else if (!property_tag.tolower().compare("technical")) {
                        file.read(technical);
                    } else if (!property_tag.tolower().compare("anatomical")) {
                        file.read(anatomical);
                    }

                    if (fromMarkers) {
                        if (!property_tag.tolower().compare("originmarkername")) {
                            file.read(originMarkerName);
                        } else if (!property_tag.tolower().compare("firstaxis")) {
                            file.read(firstAxis);
                        } else if (!property_tag.tolower().compare("firstaxismarkernames")) {
                            for (unsigned int i = 0; i<2; ++i) {
                                file.read(firstAxisMarkerNames[i]);
                            }
                        } else if (!property_tag.tolower().compare("secondaxis")) {
                            file.read(secondAxis);
                        } else if (!property_tag.tolower().compare("secondaxismarkernames")) {
                            for (unsigned int i = 0; i<2; ++i) {
                                file.read(secondAxisMarkerNames[i]);
                            }
                        } else if (!property_tag.tolower().compare("recalculate")) {
                            file.read(axisToRecalculate);
                        }
                    } else {
                        if (!property_tag.tolower().compare("rtinmatrix")) {
                            biorbd::utils::Error::check(isRTset==false,
                                                        "RT should not appear before RTinMatrix");
                            file.read(RTinMatrix);
                        } else if (!property_tag.tolower().compare("rt")) {
                            readRtMatrix(file, variable, RTinMatrix, RT);
                            isRTset = true;
                        }
                    }
                }
                if (fromMarkers) {
                    std::vector<biorbd::rigidbody::NodeSegment> allMarkerOnSegment(model->marker(
                                parent_str));
                    biorbd::rigidbody::NodeSegment origin, axis1Beg, axis1End, axis2Beg, axis2End;
                    bool isOrigin(false), isAxis1Beg(false), isAxis1End(false), isAxis2Beg(false),
                         isAxis2End(false);
                    for (auto mark : allMarkerOnSegment) {
                        if (!mark.biorbd::utils::Node::name().compare(originMarkerName)) {
                            origin = mark;
                            isOrigin = true;
                        }
                        if (!mark.biorbd::utils::Node::name().compare(firstAxisMarkerNames[0])) {
                            axis1Beg = mark;
                            isAxis1Beg = true;
                        }
                        if (!mark.biorbd::utils::Node::name().compare(firstAxisMarkerNames[1])) {
                            axis1End = mark;
                            isAxis1End = true;
                        }
                        if (!mark.biorbd::utils::Node::name().compare(secondAxisMarkerNames[0])) {
                            axis2Beg = mark;
                            isAxis2Beg = true;
                        }
                        if (!mark.biorbd::utils::Node::name().compare(secondAxisMarkerNames[1])) {
                            axis2End = mark;
                            isAxis2End = true;
                        }
                    }
                    if (! (isAxis1Beg && isAxis1End && isAxis2Beg && isAxis2End && isOrigin)) {
                        biorbd::utils::Error::raise("All the axes name and origin for the " + rtType +
                                                    "(" + name +
                                                    ") must be set and correspond to marker names previously defined on the same parent");
                    }
                    if (!(!axisToRecalculate.tolower().compare("firstaxis")
                            || !axisToRecalculate.tolower().compare("secondaxis"))) {
                        biorbd::utils::Error::raise("The 'recalculate' option for " + rtType + "(" +
                                                    name +
                                                    ") must be 'firstaxis' or 'secondaxis'");
                    }
                    axisToRecalculate = !axisToRecalculate.tolower().compare("firstaxis") ?
                                        firstAxis : secondAxis;

                    RT = biorbd::utils::RotoTrans::fromMarkers(
                             origin,
                    {axis1Beg, axis1End},
                    {axis2Beg, axis2End},
                    {firstAxis, secondAxis}, axisToRecalculate);
                }
                RT.setName(name);
                RT.setParent(parent_str);
                if (!main_tag.tolower().compare("customrt")) {
                    model->addRT(RT);
                } else {
                    model->addIMU(RT, technical, anatomical);
                }
            } else if (!main_tag.tolower().compare("contact")) {
                biorbd::utils::String name;
                file.read(name);
                unsigned int parent_int = 0;
                biorbd::utils::String parent_str("root");
                biorbd::utils::Vector3d pos(0,0,0);
                biorbd::utils::Vector3d norm(0,0,0);
                biorbd::utils::String axis("");
                double acc = 0;
                while(file.read(property_tag) && property_tag.tolower().compare("endcontact")) {
                    if (!property_tag.tolower().compare("parent")) {
                        // Dynamically find the parent number
                        file.read(parent_str);
                        parent_int = model->GetBodyId(parent_str.c_str());
                        // If parent_int equals zero, no name has concurred
                        biorbd::utils::Error::check(model->IsBodyId(parent_int),
                                                    "Wrong name in a segment");
                    } else if (!property_tag.tolower().compare("position")) {
                        readVector3d(file, variable, pos);
                    } else if (!property_tag.tolower().compare("normal")) {
                        readVector3d(file, variable, norm);
                    } else if (!property_tag.tolower().compare("axis")) {
                        file.read(axis);
                    } else if (!property_tag.tolower().compare("acceleration")) {
                        file.read(acc, variable);
                    }
                }
                if (version == 1) {
#ifndef BIORBD_USE_CASADI_MATH
                    biorbd::utils::Error::check(norm.norm() == 1.0,
                                                "Normal of the contact must be provided" );
#endif
                    model->AddConstraint(parent_int, pos, norm, name, acc);
                } else if (version >= 2) {
                    biorbd::utils::Error::check(axis.compare(""), "Axis must be provided");
                    model->AddConstraint(parent_int, pos, axis, name, acc);
                }
            } else if (!main_tag.tolower().compare("loopconstraint")) {
                biorbd::utils::String name;
                unsigned int id_predecessor = 0;
                unsigned int id_successor = 0;
                biorbd::utils::String predecessor_str("root");
                biorbd::utils::String successor_str("root");
                biorbd::utils::RotoTrans X_predecessor;
                biorbd::utils::RotoTrans X_successor;
                biorbd::utils::SpatialVector axis;
                bool enableStabilization(false);
                double stabilizationParam(-1);
                while(file.read(property_tag)
                        && property_tag.tolower().compare("endloopconstraint")) {
                    if (!property_tag.tolower().compare("predecessor")) {
                        // Dynamically find the parent number
                        file.read(predecessor_str);
                        id_predecessor = model->GetBodyId(predecessor_str.c_str());
                        // If parent_int equals zero, no name has concurred
                        biorbd::utils::Error::check(model->IsBodyId(id_predecessor),
                                                    "Wrong name in a segment");
                    }
                    if (!property_tag.tolower().compare("successor")) {
                        //  Dynamically find the parent number
                        file.read(successor_str);
                        id_successor = model->GetBodyId(successor_str.c_str());
                        // If parent_int equals zero, no name has concurred
                        biorbd::utils::Error::check(model->IsBodyId(id_successor),
                                                    "Wrong name in a segment");
                    } else if (!property_tag.tolower().compare("rtpredecessor")) {
                        biorbd::utils::String seq("xyz");
                        biorbd::utils::Vector3d rot(0, 0, 0);
                        biorbd::utils::Vector3d trans(0, 0, 0);
                        // Transcribe the rotations
                        readVector3d(file, variable, rot);
                        // Transcribe the angle sequence for the rotations
                        file.read(seq);
                        // Transcribe the translations
                        readVector3d(file, variable, trans);
                        X_predecessor = biorbd::utils::RotoTrans(rot, trans, seq);
                    } else if (!property_tag.tolower().compare("rtsuccessor")) {
                        biorbd::utils::String seq("xyz");
                        biorbd::utils::Vector3d rot(0, 0, 0);
                        biorbd::utils::Vector3d trans(0, 0, 0);
                        // Transcribe the rotations
                        readVector3d(file, variable, rot);
                        // Transcribe the angle sequence for the roations
                        file.read(seq);
                        // Transcribe the translations
                        readVector3d(file, variable, trans);
                        X_successor = biorbd::utils::RotoTrans(rot, trans, seq);
                    } else if (!property_tag.tolower().compare("axis"))
                        for (unsigned int i=0; i<axis.size(); ++i) {
                            file.read(axis(i), variable);
                        }
                    else if (!property_tag.tolower().compare("stabilizationparameter")) {
                        file.read(stabilizationParam, variable);
                    }
                }
                if (stabilizationParam > 0) {
                    enableStabilization = true;
                }
                name = "Loop_" + predecessor_str + "_" + successor_str;
                model->AddLoopConstraint(id_predecessor, id_successor, X_predecessor,
                                         X_successor,
                                         axis, name, enableStabilization, stabilizationParam);
            } else if (!main_tag.tolower().compare("actuator")) {
#ifdef MODULE_ACTUATORS
                hasActuators = true;
                // The name of the actuator must correspond to the segment number to which it is attached
                biorbd::utils::String name;
                file.read(name);
                unsigned int parent_int = model->GetBodyId(name.c_str());
                // If parent_int equals zero, no name has concurred
                biorbd::utils::Error::check(model->IsBodyId(parent_int),
                                            "Wrong name in a segment");

                // Declaration of all the parameters for all types
                biorbd::utils::String type;
                bool isTypeSet  = false;
                unsigned int dofIdx(INT_MAX);
                bool isDofSet   = false;
                biorbd::utils::String str_direction;
                bool isDirectionSet = false;
                int int_direction = 0;
                double Tmax(-1);
                bool isTmaxSet  = false;
                double T0(-1);
                bool isT0Set    = false;
                double slope(-1);
                bool isSlopeSet = false;
                double wmax(-1);
                bool iswmaxSet  = false;
                double wc(-1);
                bool iswcSet    = false;
                double amin(-1);
                bool isaminSet  = false;
                double wr(-1);
                bool iswrSet    = false;
                double w1(-1);
                bool isw1Set    = false;
                double r(-1);
                bool isrSet     = false;
                double qopt(-1);
                bool isqoptSet  = false;
                double facteur6p(-1);
                bool isFacteur6pSet = false;
                double r2(-1);
                bool isr2Set     = false;
                double qopt2(-1);
                bool isqopt2Set  = false;
                double theta(-1);
                bool isThetaSet  = false;
                double lambda(-1);
                bool isLambdaSet  = false;
                double offset(-1);
                bool isOffsetSet  = false;


                while(file.read(property_tag)
                        && property_tag.tolower().compare("endactuator")) {
                    if (!property_tag.tolower().compare("type")) {
                        file.read(type);
                        isTypeSet = true;
                    } else if (!property_tag.tolower().compare("dof")) {
                        biorbd::utils::String dofName;
                        file.read(dofName);
                        dofIdx = model->getDofIndex(name, dofName);
                        isDofSet = true;
                    } else if (!property_tag.tolower().compare("direction")) {
                        file.read(str_direction);
                        biorbd::utils::Error::check(!str_direction.tolower().compare("positive") ||
                                                    !str_direction.tolower().compare("negative"),
                                                    "Direction should be \"positive\" or \"negative\"");
                        if (!str_direction.tolower().compare("positive")) {
                            int_direction = 1;
                        } else {
                            int_direction = -1;
                        }
                        isDirectionSet = true;
                    } else if (!property_tag.tolower().compare("tmax")) {
                        file.read(Tmax, variable);
                        isTmaxSet = true;
                    } else if (!property_tag.tolower().compare("t0")) {
                        file.read(T0, variable);
                        isT0Set = true;
                    } else if (!property_tag.tolower().compare("pente")
                               || !property_tag.tolower().compare("slope")) {
                        file.read(slope, variable);
                        isSlopeSet = true;
                    } else if (!property_tag.tolower().compare("wmax")) {
                        file.read(wmax, variable);
                        iswmaxSet = true;
                    } else if (!property_tag.tolower().compare("wc")) {
                        file.read(wc, variable);
                        iswcSet = true;
                    } else if (!property_tag.tolower().compare("amin")) {
                        file.read(amin, variable);
                        isaminSet = true;
                    } else if (!property_tag.tolower().compare("wr")) {
                        file.read(wr, variable);
                        iswrSet = true;
                    } else if (!property_tag.tolower().compare("w1")) {
                        file.read(w1, variable);
                        isw1Set = true;
                    } else if (!property_tag.tolower().compare("r")) {
                        file.read(r, variable);
                        isrSet = true;
                    } else if (!property_tag.tolower().compare("qopt")) {
                        file.read(qopt, variable);
                        isqoptSet = true;
                    } else if (!property_tag.tolower().compare("facteur")) {
                        file.read(facteur6p, variable);
                        isFacteur6pSet = true;
                    } else if (!property_tag.tolower().compare("r2")) {
                        file.read(r2, variable);
                        isr2Set = true;
                    } else if (!property_tag.tolower().compare("qopt2")) {
                        file.read(qopt2, variable);
                        isqopt2Set = true;
                    } else if (!property_tag.tolower().compare("theta")) {
                        file.read(theta, variable);
                        isThetaSet = true;
                    } else if (!property_tag.tolower().compare("lambda")) {
                        file.read(lambda, variable);
                        isLambdaSet = true;
                    } else if (!property_tag.tolower().compare("offset")) {
                        file.read(offset, variable);
                        isOffsetSet = true;
                    }
                }
                // Verify that everything is there
                biorbd::utils::Error::check(isTypeSet!=0, "Actuator type must be defined");
                biorbd::actuator::Actuator* actuator;

                if (!type.tolower().compare("gauss3p")) {
                    biorbd::utils::Error::check(isDofSet && isDirectionSet && isTmaxSet && isT0Set
                                                && iswmaxSet && iswcSet && isaminSet &&
                                                iswrSet && isw1Set && isrSet && isqoptSet,
                                                "Make sure all parameters are defined");
                    actuator = new biorbd::actuator::ActuatorGauss3p(int_direction,Tmax,T0,wmax,wc,
                            amin,wr,w1,r,qopt,dofIdx,name);
                } else if (!type.tolower().compare("constant")) {
                    biorbd::utils::Error::check(isDofSet && isDirectionSet && isTmaxSet,
                                                "Make sure all parameters are defined");
                    actuator = new biorbd::actuator::ActuatorConstant(int_direction,Tmax,dofIdx,
                            name);
                } else if (!type.tolower().compare("linear")) {
                    biorbd::utils::Error::check(isDofSet && isDirectionSet && isSlopeSet && isT0Set,
                                                "Make sure all parameters are defined");
                    actuator = new biorbd::actuator::ActuatorLinear(int_direction,T0,slope,dofIdx,
                            name);
                } else if (!type.tolower().compare("gauss6p")) {
                    biorbd::utils::Error::check(isDofSet && isDirectionSet && isTmaxSet && isT0Set
                                                && iswmaxSet && iswcSet && isaminSet &&
                                                iswrSet && isw1Set && isrSet && isqoptSet && isFacteur6pSet && isr2Set
                                                && isqopt2Set,
                                                "Make sure all parameters are defined");
                    actuator = new biorbd::actuator::ActuatorGauss6p(int_direction,Tmax,T0,wmax,wc,
                            amin,wr,w1,r,qopt,facteur6p, r2, qopt2,
                            dofIdx,name);
                } else if (!type.tolower().compare("sigmoidgauss3p")) {
                    biorbd::utils::Error::check(isDofSet && isThetaSet && isLambdaSet
                                                && isOffsetSet && isrSet && isqoptSet,
                                                "Make sure all parameters are defined");
                    actuator = new biorbd::actuator::ActuatorSigmoidGauss3p(int_direction, theta,
                            lambda, offset, r, qopt, dofIdx, name);
                } else {
                    biorbd::utils::Error::raise("Actuator do not correspond to an implemented one");
#ifdef _WIN32
                    actuator = new biorbd::actuator::ActuatorConstant(int_direction, Tmax, dofIdx,
                            name); // Échec de compilation sinon
#endif
                }

                model->addActuator(*actuator);
                delete actuator;
#else // MODULE_ACTUATORS
                biorbd::utils::Error::raise("Biorbd was build without the module Actuators but the model defines ones");
#endif // MODULE_ACTUATORS
            } else if (!main_tag.tolower().compare("musclegroup")) {
#ifdef MODULE_MUSCLES
                biorbd::utils::String name;
                file.read(name); // Name of the muscular group
                // Declaration of the variables
                biorbd::utils::String origin_parent_str("root");
                biorbd::utils::String insert_parent_str("root");
                // Read the file
                while(file.read(property_tag)
                        && property_tag.tolower().compare("endmusclegroup")) {
                    if (!property_tag.tolower().compare("originparent")) {
                        // Dynamically find the parent number
                        file.read(origin_parent_str);
                        unsigned int idx = model->GetBodyId(origin_parent_str.c_str());
                        // If parent_int still equals zero, no name has concurred
                        biorbd::utils::Error::check(model->IsBodyId(idx),
                                                    "Wrong origin parent name for a muscle");
                    } else if (!property_tag.tolower().compare("insertionparent")) {
                        // Dynamically find the parent number
                        file.read(insert_parent_str);
                        unsigned int idx = model->GetBodyId(insert_parent_str.c_str());
                        // If parent_int still equals zero, no name has concurred
                        biorbd::utils::Error::check(model->IsBodyId(idx),
                                                    "Wrong insertion parent name for a muscle");
                    }
                }
                model->addMuscleGroup(name, origin_parent_str, insert_parent_str);
#else // MODULE_MUSCLES
                biorbd::utils::Error::raise("Biorbd was build without the module Muscles but the model defines a muscle group");
#endif // MODULE_MUSCLES
            } else if (!main_tag.tolower().compare("muscle")) {
#ifdef MODULE_MUSCLES
                biorbd::utils::String name;
                file.read(name); // Name of the muscle
                // Declaration of the variables
                biorbd::muscles::MUSCLE_TYPE type(biorbd::muscles::MUSCLE_TYPE::NO_MUSCLE_TYPE);
                biorbd::muscles::STATE_TYPE stateType(
                    biorbd::muscles::STATE_TYPE::NO_STATE_TYPE);
                biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType(
                    biorbd::muscles::STATE_FATIGUE_TYPE::NO_FATIGUE_STATE_TYPE);
                biorbd::utils::String muscleGroup("");
                int idxGroup(-1);
                biorbd::utils::Vector3d origin_pos(0,0,0);
                biorbd::utils::Vector3d insert_pos(0,0,0);
                double optimalLength(0);
                double maxForce(0);
                double tendonSlackLength(0);
                double pennAngle(0);
                double maxExcitation(1);
                double maxActivation(1);
                double PCSA(0);
                double shapeFactor(0);
                biorbd::muscles::FatigueParameters fatigueParameters;

                // Read file
                while(file.read(property_tag) && property_tag.tolower().compare("endmuscle")) {
                    if (!property_tag.tolower().compare("musclegroup")) {
                        // Dynamically find the parent number
                        file.read(muscleGroup);
                        idxGroup = model->getMuscleGroupId(muscleGroup);
                        // If parent_int is still equal to zero, no name has concurred
                        biorbd::utils::Error::check(idxGroup!=-1, "Could not find muscle group");
                    } else if (!property_tag.tolower().compare("type")) {
                        biorbd::utils::String tp_type;
                        file.read(tp_type);
                        if (!tp_type.tolower().compare("idealizedactuator")) {
                            type = biorbd::muscles::MUSCLE_TYPE::IDEALIZED_ACTUATOR;
                        } else if (!tp_type.tolower().compare("hill")) {
                            type = biorbd::muscles::MUSCLE_TYPE::HILL;
                        } else if (!tp_type.tolower().compare("hillthelen")
                                   || !tp_type.tolower().compare("thelen")) {
                            type = biorbd::muscles::MUSCLE_TYPE::HILL_THELEN;
                        } else if (!tp_type.tolower().compare("hillthelenactive")
                                   || !tp_type.tolower().compare("thelenactive")) {
                            type = biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_ACTIVE;
                        } else if (!tp_type.tolower().compare("hillthelenfatigable")
                                   || !tp_type.tolower().compare("thelenfatigable")) {
                            type = biorbd::muscles::MUSCLE_TYPE::HILL_THELEN_FATIGABLE;
                        } else {
                            biorbd::utils::Error::raise(property_tag + " is not a valid muscle type");
                        }
                    } else if (!property_tag.tolower().compare("statetype")) {
                        biorbd::utils::String tp_state;
                        file.read(tp_state);
                        if (!tp_state.tolower().compare("buchanan")) {
                            stateType = biorbd::muscles::STATE_TYPE::BUCHANAN;
                        } else if (!tp_state.tolower().compare("degroote")) {
                            stateType = biorbd::muscles::STATE_TYPE::DE_GROOTE;
                        } else {
                            biorbd::utils::Error::raise(property_tag + " is not a valid muscle state type");
                        }
                    } else if (!property_tag.tolower().compare("originposition")) {
                        readVector3d(file, variable, origin_pos);
                    } else if (!property_tag.tolower().compare("insertionposition")) {
                        readVector3d(file, variable, insert_pos);
                    } else if (!property_tag.tolower().compare("optimallength")) {
                        file.read(optimalLength, variable);
                    } else if (!property_tag.tolower().compare("tendonslacklength")) {
                        file.read(tendonSlackLength, variable);
                    } else if (!property_tag.tolower().compare("pennationangle")) {
                        file.read(pennAngle, variable);
                    } else if (!property_tag.tolower().compare("maximalforce")) {
                        file.read(maxForce, variable);
                    } else if (!property_tag.tolower().compare("maximalexcitation")) {
                        file.read(maxExcitation, variable);
                    } else if (!property_tag.tolower().compare("pcsa")) {
                        file.read(PCSA, variable);
                    } else if (!property_tag.tolower().compare("fatigueparameters")) {
                        while(file.read(subproperty_tag)
                                && subproperty_tag.tolower().compare("endfatigueparameters")) {
                            if (!subproperty_tag.tolower().compare("type")) {
                                biorbd::utils::String tp_fatigue_type;
                                file.read(tp_fatigue_type);
                                if (!tp_fatigue_type.tolower().compare("simple")) {
                                    dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE;
                                } else if (!tp_fatigue_type.tolower().compare("xia")) {
                                    dynamicFatigueType = biorbd::muscles::STATE_FATIGUE_TYPE::DYNAMIC_XIA;
                                } else {
                                    biorbd::utils::Error::raise(tp_fatigue_type +
                                                                " is not a value fatigue parameter type");
                                }
                            } else {
                                double param(0);
                                file.read(param);
                                if (!subproperty_tag.tolower().compare("fatiguerate")) {
                                    fatigueParameters.setFatigueRate(param);
                                } else if (!subproperty_tag.tolower().compare("recoveryrate")) {
                                    fatigueParameters.setRecoveryRate(param);
                                } else if (!subproperty_tag.tolower().compare("developfactor")) {
                                    fatigueParameters.setDevelopFactor(param);
                                } else if (!subproperty_tag.tolower().compare("recoveryfactor")) {
                                    fatigueParameters.setRecoveryFactor(param);
                                }
                            }
                        }
                    } else if (!property_tag.tolower().compare("shapefactor")) {
                        file.read(shapeFactor);
                    }
                }
                biorbd::utils::Error::check(idxGroup!=-1, "No muscle group was provided!");
                biorbd::muscles::Geometry geo(
                    biorbd::utils::Vector3d(origin_pos, name + "_origin",
                                            model->muscleGroup(static_cast<unsigned int>(idxGroup)).origin()),
                    biorbd::utils::Vector3d(insert_pos, name + "_insertion",
                                            model->muscleGroup(static_cast<unsigned int>(idxGroup)).insertion()));
                biorbd::muscles::State stateMax(maxExcitation, maxActivation);
                biorbd::muscles::Characteristics characteristics(optimalLength, maxForce, PCSA,
                        tendonSlackLength, pennAngle, stateMax,
                        fatigueParameters);
                model->muscleGroup(static_cast<unsigned int>(idxGroup)).addMuscle(name,type,geo,
                        characteristics,
                        biorbd::muscles::PathModifiers(),stateType,dynamicFatigueType);
                if (stateType == biorbd::muscles::STATE_TYPE::BUCHANAN && shapeFactor != 0) {
                    auto& muscleGroup = model->muscleGroup(idxGroup);
                    size_t nMuscInGroup(muscleGroup.nbMuscles()-1);
                    auto& state = muscleGroup.muscle(nMuscInGroup).state();
                    static_cast<biorbd::muscles::StateDynamicsBuchanan&>(state).shapeFactor(
                        shapeFactor);
                }
#else // MODULE_MUSCLES
                biorbd::utils::Error::raise("Biorbd was build without the module Muscles but the model defines a muscle");
#endif // MODULE_MUSCLES
            } else if (!main_tag.tolower().compare("viapoint")) {
#ifdef MODULE_MUSCLES
                biorbd::utils::String name;
                file.read(name); // Name of muscle... Eventually add the muscle group

                // Declaration of the variables
                biorbd::utils::String parent("");
                biorbd::utils::String muscle("");
                biorbd::utils::String musclegroup("");
                int iMuscleGroup(-1);
                int iMuscle(-1);
                biorbd::muscles::ViaPoint position(0,0,0);

                // Read file
                while(file.read(property_tag)
                        && property_tag.tolower().compare("endviapoint")) {
                    if (!property_tag.tolower().compare("parent")) {
                        // Dynamically find the parent number
                        file.read(parent);
                        unsigned int idx = model->GetBodyId(parent.c_str());
                        // If parent_int still equals zero, no name has concurred
                        biorbd::utils::Error::check(model->IsBodyId(idx),
                                                    "Wrong origin parent name for a muscle");
                    } else if (!property_tag.tolower().compare("muscle")) {
                        file.read(muscle);
                    } else if (!property_tag.tolower().compare("musclegroup")) {
                        file.read(musclegroup);
                    } else if (!property_tag.tolower().compare("position")) {
                        readVector3d(file, variable, position);
                    }
                }
                iMuscleGroup = model->getMuscleGroupId(musclegroup);
                biorbd::utils::Error::check(iMuscleGroup!=-1, "No muscle group was provided!");
                iMuscle = model->muscleGroup(static_cast<unsigned int>(iMuscleGroup)).muscleID(
                              muscle);
                biorbd::utils::Error::check(iMuscle!=-1, "No muscle was provided!");
                position.setName(name);
                position.setParent(parent);
                model->muscleGroup(static_cast<unsigned int>(iMuscleGroup))
                .muscle(static_cast<unsigned int>(iMuscle)).addPathObject(position);
#else // MODULE_MUSCLES
                biorbd::utils::Error::raise("Biorbd was build without the module Muscles but the model defines a viapoint");
#endif // MODULE_MUSCLES
            } else if (!main_tag.tolower().compare("wrapping")) {
#ifdef MODULE_MUSCLES
                biorbd::utils::String name;
                file.read(name); // Name of the wrapping

                // Declaration of the variables
                biorbd::utils::String muscle("");
                biorbd::utils::String musclegroup("");
                biorbd::utils::String wrapType("");
                int iMuscleGroup(-1);
                int iMuscle(-1);
                biorbd::utils::String parent("");
                biorbd::utils::RotoTrans RT;
                bool isRTset(false);
                bool RTinMatrix(false);
                double radius(0);
                double length(0);

                // Read file
                while(file.read(property_tag)
                        && property_tag.tolower().compare("endwrapping")) {
                    if (!property_tag.tolower().compare("parent")) {
                        // Dynamically find the parent number
                        file.read(parent);
                        unsigned int idx = model->GetBodyId(parent.c_str());
                        // If parent_int still equals zero, no name has concurred
                        biorbd::utils::Error::check(model->IsBodyId(idx),
                                                    "Wrong origin parent name for a muscle");
                    } else if (!property_tag.tolower().compare("rtinmatrix")) {
                        biorbd::utils::Error::check(isRTset==false,
                                                    "RT should not appear before RTinMatrix");
                        file.read(RTinMatrix);
                    } else if (!property_tag.tolower().compare("rt")) {
                        readRtMatrix(file, variable, RTinMatrix, RT);
                        isRTset = true;
                    } else if (!property_tag.tolower().compare("type")) {
                        file.read(wrapType);
                    } else if (!property_tag.tolower().compare("muscle")) {
                        file.read(muscle);
                    } else if (!property_tag.tolower().compare("musclegroup")) {
                        file.read(musclegroup);
                    } else if (!property_tag.tolower().compare("radius")) {
                        file.read(radius, variable);
                    } else if (!property_tag.tolower().compare("length")) {
                        file.read(length, variable);
                    }
                }
                biorbd::utils::Error::check(parent != "", "Parent was not defined");
                iMuscleGroup = model->getMuscleGroupId(musclegroup);
                biorbd::utils::Error::check(iMuscleGroup!=-1, "No muscle group was provided!");
                iMuscle = model->muscleGroup(static_cast<unsigned int>(iMuscleGroup)).muscleID(
                              muscle);
                biorbd::utils::Error::check(iMuscle!=-1, "No muscle was provided!");

                if (!wrapType.tolower().compare("halfcylinder")) {
                    biorbd::utils::Error::check(radius > 0.0,
                                                "Radius must be defined and positive");
                    biorbd::utils::Error::check(length >= 0.0, "Length was must be positive");
                    biorbd::muscles::WrappingHalfCylinder cylinder(RT,radius,length,name,parent);
                    model->muscleGroup(static_cast<unsigned int>(iMuscleGroup)).muscle(
                        static_cast<unsigned int>(iMuscle)).addPathObject(
                            cylinder);
                } else {
                    biorbd::utils::Error::raise("Wrapping type must be defined (choices: 'cylinder')");
                }
#else // MODULE_MUSCLES
                biorbd::utils::Error::raise("Biorbd was build without the module Muscles but the model defines a wrapping object");
#endif // MODULE_MUSCLES
            }
        }
    } catch (std::runtime_error message) {
        biorbd::utils::String error_message("Reading of file \"" + path.filename() + "."
                                            + path.extension() +
                                            "\" failed with the following error:");
        error_message += "\n" + biorbd::utils::String(message.what()) + "\n";
        if (name.compare("")) {
            error_message += "Element: " + main_tag + ", named: " + name + "\n";
        }
        if (property_tag.compare("") && property_tag.find_first_of("end") != 0) {
            error_message += "Property: " + property_tag + "\n";
        }
        if (subproperty_tag.compare("") && subproperty_tag.find_first_of("end") != 0) {
            error_message += "Subproperty: " + subproperty_tag + "\n";
        }

        biorbd::utils::Error::raise(error_message);
    }
#ifdef MODULE_ACTUATORS
    if (hasActuators) {
        model->closeActuator();
    }
#endif // MODULE_ACTUATORS
    // Close file
    // std::cout << "Model file successfully loaded" << std::endl;
    file.close();
}
std::vector<std::vector<biorbd::utils::Vector3d>>
        biorbd::Reader::readMarkerDataFile(
            const biorbd::utils::Path &path)
{
    // Open file
    // std::cout << "Loading marker file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
        biorbd::utils::Path::toWindowsFormat(
            path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
        path.absolutePath().c_str(), std::ios::in);
#endif

    // Read file
    biorbd::utils::String tp;

    // Determine the version of the file
    file.readSpecificTag("version", tp);
    unsigned int version(static_cast<unsigned int>(atoi(tp.c_str())));
    biorbd::utils::Error::check(version == 1, "Version not implemented yet");

    // Determine the number of markers
    file.readSpecificTag("nbmark", tp);
    unsigned int nbMark(static_cast<unsigned int>(atoi(tp.c_str())));

    // Determine the number of nodes
    file.readSpecificTag("nbintervals", tp);
    unsigned int nbIntervals(static_cast<unsigned int>(atoi(tp.c_str())));

    std::vector<std::vector<biorbd::utils::Vector3d>> markers;
    std::vector<biorbd::utils::Vector3d> position;
    // Scroll down until the definition of a marker
    for (unsigned int j=0; j<nbMark; j++) {
        while (tp.compare("Marker")) {
            bool check = file.read(tp);
            biorbd::utils::Error::check(check,
                                        "Marker file error, wrong size of marker or intervals?");
        }

        unsigned int noMarker;
        file.read(noMarker);
        for (unsigned int i=0; i<=nbIntervals;
                i++) { // <= because there is nbIntervals+1 values
            biorbd::utils::Vector3d mark(0, 0, 0);
            readVector3d(file, std::map<biorbd::utils::Equation, double>(), mark);
            position.push_back(mark);
        }
        markers.push_back(position);
        // reinitialize for the next iteration
        tp = "";
        position.clear();
    }

    // Close file
    // std::cout << "Marker file successfully loaded" << std::endl;
    file.close();
    return markers;
}

std::vector<biorbd::rigidbody::GeneralizedCoordinates>
biorbd::Reader::readQDataFile(
    const utils::Path &path)
{
    // Open file
    // std::cout << "Loading kin file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
        biorbd::utils::Path::toWindowsFormat(
            path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
        path.absolutePath().c_str(), std::ios::in);
#endif

    // Read file
    biorbd::utils::String tp;

    // Determine the file version
    file.readSpecificTag("version", tp);
    unsigned int version(static_cast<unsigned int>(atoi(tp.c_str())));
    biorbd::utils::Error::check(version == 1, "Version not implemented yet");

    // Determine the number of markers
    file.readSpecificTag("nddl", tp);
    unsigned int NDDL(static_cast<unsigned int>(atoi(tp.c_str())));

    // Determine the number of nodes
    file.readSpecificTag("nbintervals", tp);
    unsigned int nbIntervals(static_cast<unsigned int>(atoi(tp.c_str())));

    std::vector<biorbd::rigidbody::GeneralizedCoordinates> kinematics;
    // Scroll down until the definition of a marker
    for (unsigned int j=0; j<nbIntervals+1; j++) {
        while (tp.compare("T")) {
            bool check = file.read(tp);
            biorbd::utils::Error::check(check,
                                        "Kin file error, wrong size of NDDL or intervals?");
        }

        double time;
        file.read(time);
        biorbd::rigidbody::GeneralizedCoordinates position(NDDL);
        for (unsigned int i=0; i<NDDL; i++) {
            file.read(position(i));
        }

        kinematics.push_back(position);
        // reinitialise for the next iteration
        tp = "";
    }

    // Close file
    // std::cout << "Kin file successfully loaded" << std::endl;
    file.close();
    return kinematics;
}

std::vector<biorbd::utils::Vector>
biorbd::Reader::readActivationDataFile(
    const utils::Path &path)
{
    // Open file
    // std::cout << "Loading kin file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
        biorbd::utils::Path::toWindowsFormat(
            path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
        path.absolutePath().c_str(), std::ios::in);
#endif

    // Read file
    biorbd::utils::String tp;

    // Determine the file version
    file.readSpecificTag("version", tp);
    unsigned int version(static_cast<unsigned int>(atoi(tp.c_str())));
    biorbd::utils::Error::check(version == 1, "Version not implemented yet");

    // Determine the number of markers
    file.readSpecificTag("nbmuscles", tp);
    unsigned int nMus(static_cast<unsigned int>(atoi(tp.c_str())));

    // Determine the number of nodes
    file.readSpecificTag("nbintervals", tp);
    unsigned int nbIntervals(static_cast<unsigned int>(atoi(tp.c_str())));

    std::vector<biorbd::utils::Vector> activations;
    // Scroll down until the definition of a marker
    for (unsigned int j=0; j<nbIntervals+1; j++) {
        while (tp.compare("T")) {
            bool check = file.read(tp);
            biorbd::utils::Error::check(check,
                                        "Kin file error, wrong size of number of muscles or intervals?");
        }

        double time;
        file.read(time);
        biorbd::utils::Vector activation_tp(nMus);
        for (unsigned int i=0; i<nMus; i++) {
            file.read(activation_tp(i));
        }

        activations.push_back(activation_tp);
        // reinitialize for the next iteration
        tp = "";
    }

    // Close file
    // std::cout << "Activation file successfully loaded" << std::endl;
    file.close();
    return activations;
}

std::vector<biorbd::utils::Vector>
biorbd::Reader::readTorqueDataFile(
    const utils::Path &path)
{
    // Open file
    // std::cout << "Loading kin file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
        biorbd::utils::Path::toWindowsFormat(
            path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
        path.absolutePath().c_str(), std::ios::in);
#endif

    //Read file
    biorbd::utils::String tp;

    // Determine the file version
    file.readSpecificTag("version", tp);
    unsigned int version(static_cast<unsigned int>(atoi(tp.c_str())));
    biorbd::utils::Error::check(version == 1, "Version not implemented yet");

    // Determine the number of GeneralizedTorque
    file.readSpecificTag("nGeneralizedTorque", tp); //
    unsigned int nGeneralizedTorque(static_cast<unsigned int>(atoi(tp.c_str())));

    // Determine the number of nodes
    file.readSpecificTag("nbintervals", tp);
    unsigned int nbIntervals(static_cast<unsigned int>(atoi(tp.c_str())));

    std::vector<biorbd::utils::Vector> torque;
    // Scroll down until the definition of a torque
    for (unsigned int j=0; j<nbIntervals+1; j++) {
        while (tp.compare("T")) {
            bool check = file.read(tp);
            biorbd::utils::Error::check(check,
                                        "Kin file error, wrong size of NGeneralizedTorque or intervals?"); //
        }

        // Read the first line which represents the timestamp
        double time;
        file.read(time);

        // Read the vector of GeneralizedTorque associated to the timestamp
        biorbd::utils::Vector torque_tp(nGeneralizedTorque);
        for (unsigned int i=0; i<nGeneralizedTorque; i++) {
            file.read(torque_tp(i));
        }

        torque.push_back(torque_tp);
        // reinitialise for the next iteration
        tp = "";
    }

    // Close file
    file.close();
    return torque;

}

std::vector<biorbd::utils::Vector>
biorbd::Reader::readGroundReactionForceDataFile(
    const utils::Path &path)
{
    // Open file
    // std::cout << "Loading ground reaction force file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
        biorbd::utils::Path::toWindowsFormat(
            path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
        path.absolutePath().c_str(), std::ios::in);
#endif

    // Read file
    biorbd::utils::String tp;

    // Determine the file version
    file.readSpecificTag("version", tp);
    unsigned int version(static_cast<unsigned int>(atoi(tp.c_str())));
    biorbd::utils::Error::check(version == 1, "Version not implemented yet");

    // Determine the number of grf (ground reaction force)
    file.readSpecificTag("ngrf", tp); //
    unsigned int NGRF(static_cast<unsigned int>(atoi(tp.c_str())));

    // Determine the number of nodes
    file.readSpecificTag("nbintervals", tp);
    unsigned int nbIntervals(static_cast<unsigned int>(atoi(tp.c_str())));

    std::vector<biorbd::utils::Vector> grf;
    // Scroll down until the definition of a torque
    for (unsigned int j=0; j<nbIntervals+1; j++) {
        while (tp.compare("T")) {
            bool check = file.read(tp);
            biorbd::utils::Error::check(check,
                                        "Grf file error, wrong size of NR or intervals?"); //
        }

        // Read the first line that represents the timestamp
        double time;
        file.read(time);

        // Read the vector of GeneralizedTorque associated to the timestamp
        biorbd::utils::Vector grf_tp(NGRF);
        for (unsigned int i=0; i<NGRF; i++) {
            file.read(grf_tp(i));
        }

        grf.push_back(grf_tp);
        // reinitialize for the next iteration
        tp = "";
    }

    // Close file
    file.close();
    return grf;

}

void biorbd::Reader::readViconForceFile(
    const biorbd::utils::Path& path, // Path to the file
    std::vector<std::vector<unsigned int>>&
    frame, // Frame vector (time is frame/frequency)
    std::vector<unsigned int>& frequency,// Acquisition frequency
    std::vector<std::vector<biorbd::utils::Vector3d>>&
    force, // Linear forces (x,y,z)
    std::vector<std::vector<biorbd::utils::Vector3d>>& moment, // Moments (x,y,z)
    std::vector<std::vector<biorbd::utils::Vector3d>>&
    cop)  // Center of pressure (x,y,z) * number of pf
{
    // Open file
    // std::cout << "Loading force file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
        biorbd::utils::Path::toWindowsFormat(
            path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
        path.absolutePath().c_str(), std::ios::in);
#endif

    frame.clear();
    force.clear();
    moment.clear();
    cop.clear();

    // Read file
    biorbd::utils::String tp;


    while(!file.eof()) {
        // Put everything in temporary per platform (Mettre tout en temporaire par plate-forme)
        std::vector<unsigned int> frame1pf;
        std::vector<biorbd::utils::Vector3d> force1fp;
        std::vector<biorbd::utils::Vector3d> moment1fp;
        std::vector<biorbd::utils::Vector3d> cop1fp;

        // Get the acquisition frequency
        file.readSpecificTag("devices", tp);
        unsigned int frequency1pf(static_cast<unsigned int>(atoi(tp.c_str())));

        // Skip the header
        for (unsigned int i=0; i<4; ++i) {
            file.getline(tp);
        }

        // Transcribe the values
        while(!file.eof()) { // return false if it fails
            file.getline(tp); // take the entire line
            // If line is empty, it's because we are done with the forces on this platform
            if (!tp.compare("")) {
                break;
            }

            // Otherwise, we save the line
            // now we'll use a stringstream to separate the fields out of the line (comma separated)
            std::stringstream ss( tp );
            biorbd::utils::String field;
            std::vector<double> data;
            data.clear();
            while (getline( ss, field, ',' )) {
                // for each field we wish to convert it to a double
                // (since we require that the CSV contains nothing but floating-point values)
                std::stringstream fs( field );
                double f = 0.0;  // (default value is 0.0)
                fs >> f;

                // add the newly-converted field to the end of the record
                data.push_back( f );
            }
            // Make sure that the line has the right number of elements(2 times, 3 cop, 3 forces, 3 moments)
            biorbd::utils::Error::check(data.size()==11,
                                        "Wrong number of element in a line in the force file");

            // Fill in the fields
            frame1pf.push_back(static_cast<unsigned int>(data[0]));  // Frame stamp
            //        unsigned int subFrame = static_cast<unsigned int>(data[1]); // Subframes (not interesting)
            biorbd::utils::Vector3d cop_tp(0, 0, 0); // Center of pressure
            biorbd::utils::Vector3d for_tp(0, 0, 0); // Force
            biorbd::utils::Vector3d M_tp(0, 0, 0);   // Moment
            for (unsigned int i=0; i<3; ++i) {
                cop_tp[i] = data[i+2]/1000; // from mm to m
                for_tp[i] = data[i+5];
                M_tp[i] = data[i+8]/1000; // from Nmm to Nm
            }
            cop1fp.push_back(cop_tp);
            force1fp.push_back(for_tp);
            moment1fp.push_back(M_tp);
        }
        // Push back of the values
        frame.push_back(frame1pf);
        frequency.push_back(frequency1pf);
        force.push_back(force1fp);
        moment.push_back(moment1fp);
        cop.push_back(cop1fp);
    }
}

std::vector<std::vector<biorbd::utils::SpatialVector>>
        biorbd::Reader::readViconForceFile(const biorbd::utils::String &path)
{
    // Read file
    std::vector<std::vector<unsigned int>> frame;
    std::vector<unsigned int> frequency;// Acquisition frequency
    std::vector<std::vector<biorbd::utils::Vector3d>>
            force; // Linear forces (x,y,z)
    std::vector<std::vector<biorbd::utils::Vector3d>> moment; // Moments (x,y,z)
    std::vector<std::vector<biorbd::utils::Vector3d>>
            cop; // Center of pressure (x,y,z)
    readViconForceFile(path, frame, frequency, force, moment, cop);

    // Redispatch the values in a vector of SpatialTransform
    std::vector<std::vector<biorbd::utils::SpatialVector>>
            st; // nb of platform per time
    for (unsigned int j=0; j<force.size(); ++j) { // nb platform
        std::vector<biorbd::utils::SpatialVector> tp;
        for (unsigned int i=0; i<force[j].size(); ++i) { // timestamp
            const biorbd::utils::Vector3d& f = force[j][i];  // Linear forces (x,y,z)
            const biorbd::utils::Vector3d& m = moment[j][i]; // Moments (x,y,z)
            tp.push_back(biorbd::utils::SpatialVector(m[0], m[1], m[2], f[0], f[1], f[2]));
        }
        st.push_back(tp);
    }
    return st;
}

std::vector<std::vector<biorbd::utils::Vector3d>>
        biorbd::Reader::readViconMarkerFile(const biorbd::utils::Path& path,
                std::vector<biorbd::utils::String>& markOrder,
                int nFramesToGet)
{
    // Read file
#ifdef _WIN32
    biorbd::utils::IfStream file(
        biorbd::utils::Path::toWindowsFormat(
            path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
        path.absolutePath().c_str(), std::ios::in);
#endif
    biorbd::utils::String t;


    // Get the acquisition frequency
    // frequency = atoi(findImportantParameter(file, "trajectories").c_str());

    // Get the order of the markers in the file
    for (unsigned int i=0; i<3; ++i) { // skip the header
        file.read(t);
    }
    size_t idx_tp = 0;
    std::vector<unsigned int> idx_init;
    std::vector<unsigned int> idx_end;
    // Find the separators (: et ,)
    while (idx_tp < t.length()) {
        idx_tp = t.find(":", idx_tp+1);
        idx_init.push_back(static_cast<unsigned int>(idx_tp));
        idx_end.push_back(static_cast<unsigned int>(t.find(",", idx_tp+1)));
    }
    // Keep the names between the separators
    std::vector<biorbd::utils::String> MarkersInFile;
    for (unsigned int i=0; i<idx_init.size()-1; ++i) {
        biorbd::utils::String tp;
        for (unsigned int j=*(idx_init.begin()+i)+1; j<*(idx_end.begin()+i); ++j) {
            tp.push_back(t.at(j));
        }
        MarkersInFile.push_back(tp);
    }


    // Compare with the given order
    int *ordre;
    ordre = new int[3*MarkersInFile.size()];
    for (int i=0; i<static_cast<int>(3*MarkersInFile.size()); ++i) {
        ordre[i] = -1;
    }
    for (int i=0; i<static_cast<int>(markOrder.size()); ++i) {
        unsigned int cmp=0;
        biorbd::utils::String m1 = (*(markOrder.begin()+i));
        while (1) {
            biorbd::utils::String m2 = (*(MarkersInFile.begin()+cmp));
            if (!m1.compare(m2)) {
                ordre[3*cmp] = 3*i;
                ordre[3*cmp+1] = 3*i+1;
                ordre[3*cmp+2] = 3*i+2;
                break;
            } else {
                ++cmp;
            }
//            biorbd::utils::Error::check(cmp<MarkersInFile.size(), "The requested marker was not found in the file!");
            // Le marqueur demandé n'a pas été trouvé dans le fichier!
            if (cmp>=MarkersInFile.size()) {
                break;
            }
        }
    }

    // Go to the data
    for (unsigned int i=0; i<4; ++i) { // skip the header
        file.read(t);
    }

    // Find the total number of frames
    unsigned int jumps(1);
    unsigned int nbFrames(0);
    if (nFramesToGet != -1) { // If it's all of them, the jumps are 1
        while(!file.eof()) {
            file.read(t); // Get a line
            nbFrames++;
        }
        file.close();
#ifdef _WIN32
        biorbd::utils::IfStream file(
            biorbd::utils::Path::toWindowsFormat(
                path.absolutePath()).c_str(), std::ios::in);
#else
        biorbd::utils::IfStream file(
            path.absolutePath().c_str(), std::ios::in);
#endif
        // Skip the header
        for (unsigned int i=0; i<7; ++i) {
            file.read(t);
        }
        biorbd::utils::Error::check(nFramesToGet!=0 && nFramesToGet!=1
                                    && static_cast<unsigned int>(nFramesToGet)<=nbFrames,
                                    "nNode should not be 0, 1 or greater "
                                    "than number of frame");
        jumps = nbFrames/static_cast<unsigned int>(nFramesToGet)+1;
    }


    std::vector<std::vector<biorbd::utils::Vector3d>> data;
    // now we'll use a stringstream to separate the fields out of the line (comma separated)
    unsigned int cmpFrames(1);
    while(!file.eof()) {
        biorbd::utils::Vector data_tp = biorbd::utils::Vector(
                                            static_cast<unsigned int>(3*markOrder.size()));
        data_tp.setZero();

        std::stringstream ss( t );
        biorbd::utils::String field;
        unsigned int cmp = 0;
        while (getline( ss, field, ',' )) {
            // for each field we wish to convert it to a double
            // (since we require that the CSV contains nothing but floating-point values)
            std::stringstream fs( field );
            double f = 0.0;  // (default value is 0.0)
            fs >> f;
            if (cmp>1 && cmp<3*MarkersInFile.size()
                    +2) { // Retirer les timespans et ne pas dépasser
                int idx = ordre[cmp-2];
                if (idx>=0) {
                    data_tp(static_cast<unsigned int>(idx)) = f;
                }
            }
            ++cmp;
        }
        // Once the markers are in order, separate them
        std::vector<biorbd::utils::Vector3d> data_tp2;
        for (unsigned int i=0; i<static_cast<unsigned int>(data_tp.size())/3; ++i) {
            biorbd::utils::Vector3d node(data_tp.block(3*i, 0, 3, 1)/1000);
            data_tp2.push_back(node);
        }
        // Stock the marker vector a that time t
        data.push_back(data_tp2); // Put back to meters
        for (unsigned int i=0; i<jumps; ++i) {
            if (cmpFrames != nbFrames) {
                file.read(t); // Get the line
                cmpFrames++;
            } else {
                file.read(t);
                break;
            }
        }
    }

    // Close file
    file.close();

    return data;
}

biorbd::rigidbody::Mesh
biorbd::Reader::readMeshFileBiorbdSegments(
    const biorbd::utils::Path &path)
{
    // Read a segment file

    // Open file
    // std::cout << "Loading marker file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
        biorbd::utils::Path::toWindowsFormat(
            path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
        path.absolutePath().c_str(), std::ios::in);
#endif

    // Read file
    biorbd::utils::String tp;

    // Determine the file version
    file.readSpecificTag("version", tp);
    unsigned int version(static_cast<unsigned int>(atoi(tp.c_str())));
    biorbd::utils::Error::check(version == 1
                                || version == 2, "Version not implemented yet");

    // Know the number of points
    file.readSpecificTag("npoints", tp);
    unsigned int nPoints(static_cast<unsigned int>(atoi(tp.c_str())));
    file.readSpecificTag("nfaces", tp);
    unsigned int nFaces(static_cast<unsigned int>(atoi(tp.c_str())));

    biorbd::rigidbody::Mesh mesh;
    mesh.setPath(path);
    // Get all the points
    std::map<biorbd::utils::Equation, double> variable =
        std::map<biorbd::utils::Equation, double>();
    biorbd::utils::Vector3d dump; // Ignore columns 4 5 6
    for (unsigned int iPoints=0; iPoints < nPoints; ++iPoints) {
        biorbd::utils::Vector3d nodeTp(0, 0, 0);
        readVector3d(file, variable, nodeTp);
        mesh.addPoint(nodeTp);
        if (version == 2) {
            readVector3d(file, variable, dump);
        }
    }

    for (unsigned int iPoints=0; iPoints < nFaces; ++iPoints) {
        biorbd::rigidbody::MeshFace patchTp;
        unsigned int nVertices;
        file.read(nVertices);
        if (nVertices != 3) {
            biorbd::utils::Error::raise("Patches must be 3 vertices!");
        }
        for (unsigned int i=0; i<nVertices; ++i) {
            file.read(patchTp(i));
        }
        mesh.addFace(patchTp);
    }
    return mesh;
}


biorbd::rigidbody::Mesh biorbd::Reader::readMeshFilePly(
    const biorbd::utils::Path &path)
{
    // Read a bone file

    // Open file
    // std::cout << "Loading marker file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
        biorbd::utils::Path::toWindowsFormat(
            path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
        path.absolutePath().c_str(), std::ios::in);
#endif

    // Read file
    biorbd::utils::String tp;

    // Know the number of points
    file.reachSpecificTag("element");
    file.readSpecificTag("vertex", tp);
    unsigned int nVertex(static_cast<unsigned int>(atoi(tp.c_str())));
    int nVertexProperties(file.countTagsInAConsecutiveLines("property"));

    // Find the number of columns for the vertex
    file.reachSpecificTag("element");
    file.readSpecificTag("face", tp);
    unsigned int nFaces(static_cast<unsigned int>(atoi(tp.c_str())));
    int nFacesProperties(file.countTagsInAConsecutiveLines("property"));


    // Trouver le nombre de ??
    file.reachSpecificTag("end_header");

    biorbd::rigidbody::Mesh mesh;
    mesh.setPath(path);
    std::map<biorbd::utils::Equation, double> variable =
        std::map<biorbd::utils::Equation, double>();
    // Get all the points
    for (unsigned int iPoints=0; iPoints < nVertex; ++iPoints) {
        biorbd::utils::Vector3d nodeTp(0, 0, 0);
        readVector3d(file, variable, nodeTp);
        mesh.addPoint(nodeTp);
        // Ignore the columns post XYZ
        for (int i=0; i<nVertexProperties-3; ++i) {
            double dump;
            file.read(dump);
        }
    }

    for (unsigned int iPoints=0; iPoints < nFaces; ++iPoints) {
        biorbd::rigidbody::MeshFace patchTp;
        unsigned int nVertices;
        file.read(nVertices);
        if (nVertices != 3) {
            biorbd::utils::Error::raise("Patches must be 3 vertices!");
        }
        for (unsigned int i=0; i<nVertices; ++i) {
            file.read(patchTp(i));
        }
        int dump;
        // Remove if there are too many columns
        for (int i=0; i<nFacesProperties-1; ++i) {
            file.read(dump);
        }
        mesh.addFace(patchTp);
    }
    return mesh;
}

biorbd::rigidbody::Mesh biorbd::Reader::readMeshFileObj(
    const biorbd::utils::Path &path)
{
    // Read a bone file

    // Open file
    // std::cout << "Loading marker file: " << path << std::endl;
#ifdef _WIN32
    biorbd::utils::IfStream file(
        biorbd::utils::Path::toWindowsFormat(
            path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
        path.absolutePath().c_str(), std::ios::in);
#endif

    // Read file
    biorbd::utils::String tp;

    biorbd::rigidbody::Mesh mesh;
    mesh.setPath(path);

    // Get all the points
    biorbd::utils::Vector3d vertex;
    biorbd::rigidbody::MeshFace patch;
    biorbd::utils::String text;
    std::map<biorbd::utils::Equation, double> variable =
        std::map<biorbd::utils::Equation, double>();
    while (true) {
        // If we get to the end of file, exit loop
        if (file.eof()) {
            break;
        }

        // Read the first element of the line
        file.read(text);

        if (!text.compare("v")) {
            // If first element is a v, then a vertex is found
            readVector3d(file, variable, vertex);
            mesh.addPoint(vertex);
        } else if (!text.compare("f")) {
            // If first element is a f, then a face is found
            // Face is ignore for now
            for (unsigned int i=0; i<3; ++i) {
                file.read(text);
                size_t idxSlash = text.find("/");
                biorbd::utils::String tata3(text.substr (0,idxSlash));
                patch(i) = (std::stoi(text.substr (0,idxSlash)) - 1);
            }
            file.getline(text); // Ignore last element if it is a 4 vertex based
            mesh.addFace(patch.DeepCopy());
        } else {
            // Ignore the line
            file.getline(text);
        }

    }

    return mesh;
}

#ifdef MODULE_VTP_FILES_READER
#include "tinyxml.h"
biorbd::rigidbody::Mesh biorbd::Reader::readMeshFileVtp(
    const biorbd::utils::Path &path)
{
    // Read an opensim formatted mesh file

    // Read the file
#ifdef _WIN32
    biorbd::utils::String filepath( biorbd::utils::Path::toWindowsFormat(
                                        path.absolutePath()).c_str());
#else
    biorbd::utils::String filepath( path.absolutePath().c_str() );
#endif

    TiXmlDocument doc(filepath);
    biorbd::utils::Error::check(doc.LoadFile(), "Failed to load file " + filepath);
    TiXmlHandle hDoc(&doc);
    biorbd::rigidbody::Mesh mesh;
    mesh.setPath(path);

    // Navigate up to VTKFile/PolyData/Piece
    TiXmlHandle polydata =
        hDoc.ChildElement("VTKFile", 0)
        .ChildElement("PolyData", 0)
        .ChildElement("Piece", 0);
    int numberOfPoints, NumberOfPolys;
    polydata.ToElement()->QueryIntAttribute("NumberOfPoints", &numberOfPoints);
    polydata.ToElement()->QueryIntAttribute("NumberOfPolys", &NumberOfPolys);

    biorbd::utils::String field;

    // Get the points
    {
        biorbd::utils::String points(
            polydata.ChildElement("Points", 0)
            .ChildElement("DataArray", 0).Element()->GetText());
        std::stringstream ss( points );
        for (int i = 0; i < numberOfPoints; ++i) {
            double x, y, z;
            {
                getline( ss, field, ' ' );
                std::stringstream fs( field );
                fs >> x;
            }
            {
                getline( ss, field, ' ' );
                std::stringstream fs( field );
                fs >> y;
            }
            {
                getline( ss, field, ' ' );
                std::stringstream fs( field );
                fs >> z;
            }
            mesh.addPoint(biorbd::utils::Vector3d(x, y, z));
        }
    }

    // Get the patches
    {
        biorbd::utils::String polys(
            polydata.ChildElement("Polys", 0)
            .ChildElement("DataArray", 0).Element()->GetText());
        std::stringstream ss( polys );
        for (int i = 0; i < NumberOfPolys; ++i) {
            int vertex1, vertex2, vertex3;
            {
                getline( ss, field, ' ' );
                std::stringstream fs( field );
                fs >> vertex1;
            }
            {
                getline( ss, field, ' ' );
                std::stringstream fs( field );
                fs >> vertex2;
            }
            {
                getline( ss, field, ' ' );
                std::stringstream fs( field );
                fs >> vertex3;
            }
            mesh.addFace({vertex1, vertex2, vertex3});
        }
    }

    return mesh;
}

void biorbd::Reader::readVector3d(
    biorbd::utils::IfStream &file,
    const std::map<biorbd::utils::Equation, double> &variable,
    biorbd::utils::Vector3d &vector)
{
    for (unsigned int i=0; i<3; ++i) {
        file.read(vector(i), variable);
    }
}

void biorbd::Reader::readMatrix33(
    biorbd::utils::IfStream &file,
    const std::map<biorbd::utils::Equation, double> &variable,
    RigidBodyDynamics::Math::Matrix3d &matrix)
{
    for (unsigned int i=0; i<3; ++i) {
        for (unsigned int j=0; j<3; ++j) {
            file.read(matrix(i, j), variable);
        }
    }
}

void biorbd::Reader::readRtMatrix(
    biorbd::utils::IfStream& file,
    const std::map<biorbd::utils::Equation, double>& variable,
    bool RTinMatrix,
    biorbd::utils::RotoTrans &RT)
{
    if (RTinMatrix) { // Matrix 4x4
        // Counter for classification (Compteur pour classification)
        for (unsigned int i=0; i<4; ++i) {
            for (unsigned int j=0; j<4; ++j) {
                file.read(RT(i, j), variable);
            }
        }
    } else {
        biorbd::utils::String seq("xyz");
        biorbd::utils::Vector3d rot(0, 0, 0);
        biorbd::utils::Vector3d trans(0, 0, 0);
        // Transcribe the rotations
        for (unsigned int i=0; i<3; ++i) {
            file.read(rot(i));
        }
        // Transcribe the angular sequence for the rotation
        file.read(seq);
        //Transcribe the translations
        for (unsigned int i=0; i<3; ++i) {
            file.read(trans(i));
        }
        RT = biorbd::utils::RotoTrans(rot, trans, seq);
    }
}
#endif  // MODULE_VTP_FILES_READER


std::vector<std::vector<biorbd::utils::Vector3d>>
        biorbd::Reader::readViconMarkerFile(const biorbd::utils::Path &path,
                int nFramesToGet)
{
    // Read file
#ifdef _WIN32
    biorbd::utils::IfStream file(
        biorbd::utils::Path::toWindowsFormat(
            path.absolutePath()).c_str(), std::ios::in);
#else
    biorbd::utils::IfStream file(
        path.absolutePath().c_str(), std::ios::in);
#endif
    biorbd::utils::String t;


    // Get the acquisition frequency
    // frequency = atoi(findImportantParameter(file, "trajectories").c_str());

    // Get the order of the markers in the file
    for (unsigned int i=0; i<3; ++i) { // skip the header
        file.read(t);
    }
    size_t idx_tp = 0;
    std::vector<unsigned int> idx_init;
    std::vector<unsigned int> idx_end;
    // Find the separators (: et ,)
    while (idx_tp < t.length()) {
        idx_tp = t.find(":", idx_tp+1);
        idx_init.push_back(static_cast<unsigned int>(idx_tp));
        idx_end.push_back(static_cast<unsigned int>(t.find(",", idx_tp+1)));
    }
    // Keep the names between the separators
    std::vector<biorbd::utils::String> MarkersInFile;
    for (unsigned int i=0; i<idx_init.size()-1; ++i) {
        biorbd::utils::String tp;
        for (unsigned int j=*(idx_init.begin()+i)+1; j<*(idx_end.begin()+i); ++j) {
            tp.push_back(t.at(j));
        }
        MarkersInFile.push_back(tp);
    }

    // Close file
    file.close();

    return readViconMarkerFile(path, MarkersInFile, nFramesToGet);
}
