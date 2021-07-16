#ifndef BIORBD_UTILS_READ_H
#define BIORBD_UTILS_READ_H

#include <vector>
#include <map>
#include "biorbdConfig.h"
#include "rbdl/rbdl_math.h"

namespace biorbd
{
class Model;

namespace rigidbody
{
class GeneralizedCoordinates;
class Mesh;
}

namespace utils
{
class Path;
class String;
class Vector3d;
class Vector;
class SpatialVector;
class RotoTrans;
class IfStream;
class Equation;
}

///
/// \brief Reader for bioMod files
///
class BIORBD_API Reader
{
public:
    ///
    /// \brief Create a biorbd model from a bioMod file
    /// \param path The path of the file
    ///
    static biorbd::Model readModelFile(
        const biorbd::utils::Path &path);

    ///
    /// \brief Create a biorbd model from a bioMod file
    /// \param path The path of the file
    /// \param model The model to fill
    /// \return Returns the model to fill
    ///
    static void readModelFile(
        const biorbd::utils::Path &path,
        biorbd::Model *model);

    ///
    /// \brief Read a bioMark file, containing markers data
    /// \param path The path of the file
    /// \return Returns the markers
    ///
    static std::vector<std::vector<biorbd::utils::Vector3d>> readMarkerDataFile(
                const utils::Path &path);

    ///
    /// \brief Read a bioKin file, containing kinematics data
    /// \param path The path of the file
    /// \return Returns the kinematics
    ///
    static std::vector<biorbd::rigidbody::GeneralizedCoordinates> readQDataFile(
        const biorbd::utils::Path &path);

    ///
    /// \brief Read a bioMus fine, containing muscle activations data
    /// \param path The path of the file
    /// \return Returns the activations
    ///
    static std::vector<biorbd::utils::Vector> readActivationDataFile(
        const biorbd::utils::Path &path);

    ///
    /// \brief Read a bioTorque file containing generalized torques data
    /// \param path The path of the file
    /// \return Returns the torque
    ///
    static std::vector<biorbd::utils::Vector> readTorqueDataFile(
        const biorbd::utils::Path &path);

    ///
    /// \brief Read a bioGRF file containing ground reaction force (GRF) data
    /// \param path The path of the file
    /// \return Returns the ground reaction force
    ///
    static std::vector<biorbd::utils::Vector> readGroundReactionForceDataFile(
        const biorbd::utils::Path &path);

    ///
    /// \brief Read a Vicon ASCII force file
    /// \param path The path of the file
    /// \param frame The fame vector (output)
    /// \param frequency The acquisition frequency (output)
    /// \param force The linear forces (x,y,z) (output)
    /// \param moment The moments (x,y,z) (output)
    /// \param cop The center of pressure (x,y,z) (output)
    ///
    static void readViconForceFile(
        const biorbd::utils::Path &path,
        std::vector<std::vector<unsigned int>> &frame,
        std::vector<unsigned int> &frequency,
        std::vector<std::vector<biorbd::utils::Vector3d>> &force,
        std::vector<std::vector<biorbd::utils::Vector3d>> &moment,
        std::vector<std::vector<biorbd::utils::Vector3d>> &cop);

    ///
    /// \brief Read a Vicon ASCII force file
    /// \param path The path of the file
    /// \return Returns all the data in a spatial transform format
    ///
    static std::vector<std::vector<biorbd::utils::SpatialVector>>
            readViconForceFile(
                const biorbd::utils::String &path);

    ///
    /// \brief Read a Vicon ASCII marker file (CSV formated)
    /// \param path The path of the file
    /// \param nFramesToGet The number of frames
    ///
    /// If the number of frame asked is less than the total number of frames,
    /// frames are evenly skip while reading. To get all frames, nFramesToGet
    /// should be set to -1.
    ///
    static std::vector<std::vector<biorbd::utils::Vector3d>>
            readViconMarkerFile(
                const biorbd::utils::Path &path,
                int nFramesToGet =-1);

    ///
    /// \brief Read a Vicon ASCII marker file (CSV formated)
    /// \param path The path of the file
    /// \param markOrder The markers to keep
    /// \param nFramesToGet The number of frames
    /// \return Returns the data
    ///
    /// markOrder can be used for both changing the order of the marker to read
    /// and to skip some markers
    ///
    /// If the number of frame asked is less than the total number of frames,
    /// frames are evenly skip while reading. To get all frames, nFramesToGet
    /// should be set to -1.
    ///
    static std::vector<std::vector<biorbd::utils::Vector3d>>  readViconMarkerFile(
                const biorbd::utils::Path &path,
                std::vector<biorbd::utils::String> &markOrder,
                int nFramesToGet = -1);

    ///
    /// \brief Read a bioMesh file containing the meshing of a segment
    /// \param path The path of the file
    /// \return Returns the mesh
    ///
    static biorbd::rigidbody::Mesh readMeshFileBiorbdSegments(
        const biorbd::utils::Path& path);

    ///
    /// \brief Read a PLY file containing the meshing of a segment
    /// \param path The path of the file
    /// \return Returns the mesh
    ///
    static biorbd::rigidbody::Mesh readMeshFilePly(
        const biorbd::utils::Path& path);

    ///
    /// \brief Read a OBJ file containing the meshing of a segment
    /// \param path The path of the file
    /// \return Returns the mesh
    ///
    static biorbd::rigidbody::Mesh readMeshFileObj(
        const biorbd::utils::Path& path);

#ifdef MODULE_VTP_FILES_READER
    ///
    /// \brief Read a VTP (OpenSim) file containing the meshing of a segment
    /// \param path The path of the file
    /// \return Returns the mesh
    ///
    static biorbd::rigidbody::Mesh readMeshFileVtp(
        const biorbd::utils::Path& path);
#endif

protected:
    ///
    /// \brief Read a Vector 3d
    /// \param file A reference to the current file being read pointing to the RT
    /// \param variable The variable set
    /// \param vector The vector to fill
    ///
    static void readVector3d(
        biorbd::utils::IfStream& file,
        const std::map<biorbd::utils::Equation, double>& variable,
        biorbd::utils::Vector3d &vector);

    ///
    /// \brief Read a Matrix 3x3
    /// \param file A reference to the current file being read pointing to the RT
    /// \param variable The variable set
    /// \param matrix The matrix to fill
    ///
    static void readMatrix33(
        biorbd::utils::IfStream& file,
        const std::map<biorbd::utils::Equation, double>& variable,
        RigidBodyDynamics::Math::Matrix3d &matrix);

    ///
    /// \brief Read a RT matrix either in 4x4 format or Rot1233 seq Trans123 format
    /// \param file A reference to the current file being read pointing to the RT
    /// \param variable The variable set
    /// \param RTinMatrix If the RT is in matrix or in Euler format
    /// \param RT_R The Rotation part to fill
    /// \param RT_T The translation part to fill
    ///
    static void readRtMatrix(
        biorbd::utils::IfStream& file,
        const std::map<biorbd::utils::Equation, double>& variable,
        bool RTinMatrix,
        biorbd::utils::RotoTrans &RT);
};

}

#endif // BIORBD_UTILS_READ_H
