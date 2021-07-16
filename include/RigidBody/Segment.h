#ifndef BIORBD_RIGIDBODY_SEGMENT_H
#define BIORBD_RIGIDBODY_SEGMENT_H

#include <vector>
#include <rbdl/Model.h>
#include <rbdl/Joint.h>
#include "biorbdConfig.h"
#include "Utils/Node.h"

namespace biorbd
{
namespace utils
{
class RotoTrans;
class Range;
}

namespace rigidbody
{
class Joints;
class SegmentCharacteristics;

///
/// \brief Description of a segment
///
class BIORBD_API Segment : public biorbd::utils::Node
{
public:
    ///
    /// \brief Construct a Segment
    ///
    Segment();

    ///
    /// \brief Construct a Segment
    /// \param model The joint model
    /// \param name The name of the segment
    /// \param parentName The name of the parent segment
    /// \param seqT Sequence of the translations
    /// \param seqR Angle sequence of the Euler rotations
    /// \param QRanges Ranges of the translations and rotations dof. The length of QRanges must be equal to length of translations and rotations
    /// \param QDotRanges Ranges of the translations and rotations dof velocity. The length of QDotRanges must be equal to length of translations and rotations
    /// \param QDDotRanges Ranges of the translations and rotations dof acceleration. The length of QDDotRanges must be equal to length of translations and rotations
    /// \param characteristics of the segment (mass, center of mass, inertia, etc.)
    /// \param cor Transformation in parent reference frame
    /// \param PF Platform index attached to the body (-1 means no force platform acts on the body)
    ///
    Segment(
        biorbd::rigidbody::Joints& model,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName,
        const biorbd::utils::String &seqT,
        const biorbd::utils::String &seqR,
        const std::vector<biorbd::utils::Range>& QRanges,
        const std::vector<biorbd::utils::Range>& QDotRanges,
        const std::vector<biorbd::utils::Range>& QDDotRanges,
        const biorbd::rigidbody::SegmentCharacteristics& characteristics,
        const RigidBodyDynamics::Math::SpatialTransform& cor,
        int PF = -1);

    ///
    /// \brief Construct a Segment
    /// \param model The joint model
    /// \param name The name of the segment
    /// \param parentName The name of the parent segment
    /// \param seqR Angle sequence of the Euler rotations
    /// \param QRanges Ranges of the translations and rotations dof. The length of QRanges must be equal to length of translations and rotations
    /// \param QDotRanges Ranges of the translations and rotations dof velocity. The length of QDotRanges must be equal to length of translations and rotations
    /// \param QDDotRanges Ranges of the translations and rotations dof acceleration. The length of QDDotRanges must be equal to length of translations and rotations
    /// \param characteristics of the segment (mass, center of mass, inertia, etc.)
    /// \param cor Transformation in parent reference frame
    /// \param PF Platform index attached to the body (-1 means no force platform acts on the body)
    ///
    Segment(
        biorbd::rigidbody::Joints& model,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName,
        const biorbd::utils::String &seqR,
        const std::vector<biorbd::utils::Range>& QRanges,
        const std::vector<biorbd::utils::Range>& QDotRanges,
        const std::vector<biorbd::utils::Range>& QDDotRanges,
        const biorbd::rigidbody::SegmentCharacteristics& characteristics,
        const RigidBodyDynamics::Math::SpatialTransform& cor,
        int PF = -1);

    ///
    /// \brief Create a deep copy of Segment
    /// \return Copy of Segment
    ///
    biorbd::rigidbody::Segment DeepCopy() const;

    ///
    /// \brief Deep copy of Segment
    /// \param other The Segment to copy
    ///
    void DeepCopy(
        const biorbd::rigidbody::Segment& other);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~Segment();

    ///
    /// \brief Return the Segment index
    /// \return The Segment index
    ///
    unsigned int id() const;

    ///
    /// \brief Return the platform index
    /// \return The platform index
    ///
    int platformIdx() const;

    ///
    /// \brief Return the translation sequence in text
    /// \return The translation sequence in text
    ///
    const biorbd::utils::String& seqT() const;

    ///
    /// \brief Return the angle sequence in text
    /// \return The angle sequence in text
    ///
    const biorbd::utils::String& seqR() const;

    ///
    /// \brief Return the ranges for all the dof, translations and rotations respectively
    /// \return The ranges for all the dof, translations and rotations respectively
    ///
    const std::vector<biorbd::utils::Range>&
    QRanges() const;

    ///
    /// \brief Return the ranges for all the dof velocity, translations and rotations respectively
    /// \return The ranges for all the dof velocity, translations and rotations respectively
    ///
    const std::vector<biorbd::utils::Range>&
    QDotRanges() const;

    ///
    /// \brief Return the ranges for all the dof acceleration, translations and rotations respectively
    /// \return The ranges for all the dofa acceleration, translations and rotations respectively
    ///
    const std::vector<biorbd::utils::Range>&
    QDDotRanges() const;

    ///
    /// \brief Return the number of DoF of the segment
    /// \return The number of Dof of the segment
    ///
    unsigned int nbDof() const;

    ///
    /// \brief Return the number of translation DoF of the segment
    /// \return The number of translation DoF of the segment
    ///
    unsigned int nbDofTrans() const;

    ///
    /// \brief Return the number of rotation DoF of the segment
    /// \return The number of rotation DoF of the segment
    ///
    unsigned int nbDofRot() const;

    ///
    /// \brief Return the number of generalized coordinates
    /// \return The number of generalized coordinates
    ///
    unsigned int nbQ() const;

    ///
    /// \brief Return the number of generalized velocities
    /// \return The number of generalized velocities
    ///
    unsigned int nbQdot() const;

    ///
    /// \brief Return the number of generalized accelerations
    /// \return The number of generalized accelerations
    ///
    unsigned int nbQddot() const;

    ///
    /// \brief Return the number of generalized torque
    /// \return The number of generalized torque
    ///
    /// This value is equal to nbQddot
    ///
    unsigned int nbGeneralizedTorque() const;

    ///
    /// \brief Return the index of a specified DoF
    /// \return The index of a specified DoF
    ///
    unsigned int getDofIdx(
        const biorbd::utils::String &dofName) const;

    ///
    /// \brief Return the name of the specified DoF
    /// \return The name of the specified DoF
    ///
    const biorbd::utils::String& nameDof(
        const unsigned int i) const;

    ///
    /// \brief Return the joint coordinate system (JCS) in the parent reference frame
    /// \return The joint coordinate system in the parent reference frame
    ///
    biorbd::utils::RotoTrans localJCS() const;

    ///
    /// \brief updateCharacteristics Change the inertia characteristics of the segment
    /// \param model The underlying model to update
    /// \param characteristics The new characteristics
    ///
    /// Warning: This function may behave surpringly due to the core of RBDL. The
    /// new characteristic values will replace everything which is attach in a fixed manner
    /// (that is no degrees-of-freedom). So if your model has 3 segments, but only the first
    /// one has dof (and the rest is rigidly attached to the first), then it doesn't matter
    /// if idx is 0, 1 or 2, because RBDL considers that all these segment are 1 segment.
    /// It is therefore expected that characteristics is the combination of mass and
    /// inertia for these 3 segments as well.
    ///
    void updateCharacteristics(
        biorbd::rigidbody::Joints& model,
        const biorbd::rigidbody::SegmentCharacteristics& characteristics);

    ///
    /// \brief Return the segment characteristics
    /// \return The segment characteristics
    ///
    const biorbd::rigidbody::SegmentCharacteristics& characteristics() const;

    ///
    /// \brief Return if the rotation DoF of this segment is a quaternion
    /// \return If the rotation DoF of this segment is a quaternion
    ///
    bool isRotationAQuaternion() const;

protected:
    std::shared_ptr<int> m_idxInModel; ///< Index in RBDL model

    ///
    /// \brief Set the type of the segment
    ///
    void setType();

    std::shared_ptr<int> m_idxPF; ///< Platform index which acts on the segment

    ///
    /// \brief Set the platform index
    ///
    void setPF(
        int idx);

    std::shared_ptr<RigidBodyDynamics::Math::SpatialTransform>
    m_cor; ///< Attitude of the segment in parent reference frame

    ///
    /// \brief Set the DoF
    /// \param model The joint model
    /// \param seqT Sequence of the translations
    /// \param seqR Angle sequence of the Euler rotations
    /// \param QRanges Ranges of the translations and rotations dof. The length of QRanges must be equal to length of translations and rotations
    /// \param QDotRanges Ranges of the translations and rotations dof velocity. The length of QDotRanges must be equal to length of translations and rotations
    /// \param QDDotRanges Ranges of the translations and rotations dof acceleration. The length of QDDotRanges must be equal to length of translations and rotations
    ///
    void setDofs(
        biorbd::rigidbody::Joints& model,
        const biorbd::utils::String &seqT,
        const biorbd::utils::String &seqR,
        const std::vector<biorbd::utils::Range>& QRanges,
        const std::vector<biorbd::utils::Range>& QDotRanges,
        const std::vector<biorbd::utils::Range>& QDDotRanges);

    ///
    /// \brief Set the total number of DoF
    /// \param nbTrans Number of translation DoF
    /// \param nbRot Number of rotation DoF
    ///
    void setNumberOfDof(
        unsigned int nbTrans,
        unsigned int nbRot);

    std::shared_ptr<biorbd::utils::String> m_seqT;  ///< Translation sequence
    std::shared_ptr<biorbd::utils::String> m_seqR;  ///< Euler rotation sequence
    std::shared_ptr<std::vector<biorbd::utils::Range>>
            m_QRanges;  ///< Minimum and maximum coordinate values that each dof should hold. This is only prescriptive and can be ignored when setting the GeneralizedCoordinates
    std::shared_ptr<std::vector<biorbd::utils::Range>>
            m_QDotRanges;  ///< Minimum and maximum velocity values that each dof should hold. This is only prescriptive and can be ignored when setting the GeneralizedVelocities
    std::shared_ptr<std::vector<biorbd::utils::Range>>
            m_QDDotRanges;  ///< Minimum and maximum acceleration values that each dof should hold. This is only prescriptive and can be ignored when setting the GeneralizedAccelerations
    std::shared_ptr<unsigned int> m_nbDof;   ///< Number of degrees of freedom
    std::shared_ptr<unsigned int> m_nbQdot;  ///< Number of generalized velocities
    std::shared_ptr<unsigned int>
    m_nbQddot;  ///< Number of generalized accelerations
    std::shared_ptr<unsigned int>
    m_nbDofTrue;    ///< Number of degrees of freedom including the extra DoF when there is a quaternion
    std::shared_ptr<unsigned int>
    m_nbDofTrueOutside; ///< Number of degree of freedom read from the outside (Same as nDof except if Quaternion)
    std::shared_ptr<unsigned int>
    m_nbDofTrans; ///< Number of degrees of freedom in translation
    std::shared_ptr<unsigned int>
    m_nbDofRot; ///< Number of degrees of freedom in rotation
    std::shared_ptr<unsigned int>
    m_nbDofQuat; ///< Number of degrees of freedom in rotation if expressed in quaternion

    std::shared_ptr<bool> m_isQuaternion; ///< If DoF in rotation is a Quaternion

    ///
    /// \brief Determines if DoF in rotation is a Quaternion
    /// \param seqR Cardan sequence to classify the DoF in rotation
    ///
    /// If seqR is equal to "q" then it is a quaternion
    ///
    void determineIfRotIsQuaternion(const biorbd::utils::String &seqR);

    std::shared_ptr<std::vector<RigidBodyDynamics::Joint>>
            m_dof; ///< Actual DoF: t1, t2, t3, r1, r2, r3; where the order depends on seqT and seqR
    std::shared_ptr<std::vector<unsigned int>>
                                            m_idxDof;  ///< Index of the parent segment

    ///
    /// \brief Set angle and translation sequences, adjust angle sequence and redeclare if is necessary
    /// \param seqT Sequence of the translations
    /// \param seqR Angle sequence of the Euler rotations
    ///
    void setSequence(
        const biorbd::utils::String &seqT,
        const biorbd::utils::String &seqR);

    ///
    /// \brief Fill the transation and rotation sequences
    ///
    /// Places the translations first, followed by the rotations in the asked order.
    ///
    void fillSequence();

    ///
    /// \brief Convert a text sequence to its number counterpart (x = 0, y = 1, z = 2, q = 3)
    /// \param sequenceInteger Initialized vector into which the results should be placed in
    /// \param sequenceText The sequence to convert
    ///
    void str2numSequence(
        std::vector<unsigned int> &sequenceInteger,
        const biorbd::utils::String &sequenceText);

    ///
    /// \brief Store the sequences
    /// \param seqT Sequence of the translations
    /// \param seqR Angle sequence of the Euler rotations
    ///
    void str2numSequence(
        const biorbd::utils::String &seqT,
        const biorbd::utils::String &seqR);

    std::shared_ptr<std::vector<unsigned int>>
                                            m_sequenceTrans; ///< Translation sequence
    std::shared_ptr<std::vector<unsigned int>>
                                            m_sequenceRot; ///< Euler rotation sequence
    std::shared_ptr<std::vector<biorbd::utils::String>>
            m_nameDof; ///< To store the DoF names

    ///
    /// \brief Function that adds the segment to the RBDL body set
    /// \param model The joint model
    ///
    virtual void setJoints(biorbd::rigidbody::Joints& model);

    ///
    /// \brief Determine the rotation axis in relation to the requested sequence
    ///
    virtual void setJointAxis();

    std::shared_ptr<std::vector<unsigned int>>
                                            m_dofPosition;  ///< Position in the x, y, and z sequence

    ///
    /// \brief Set the DoF segment characteristics on the last body
    ///
    /// The idea is that since a segment is described by all of its DoF, the inertia
    /// and masses must be put on the last body
    ///
    void setDofCharacteristicsOnLastBody();

    std::shared_ptr<biorbd::rigidbody::SegmentCharacteristics>
    m_characteristics;///< Non-used virtual segment; it allows to "save" the data and to avoid the use of multiple intermediate variables
    std::shared_ptr<std::vector<biorbd::rigidbody::SegmentCharacteristics>>
            m_dofCharacteristics;  ///< Variable containing the inertial data and other from each segment (on a 6DoF segment, 0 to 4 should be empty and 5 filled)


};

}
}

#endif // BIORBD_RIGIDBODY_SEGMENT_H
