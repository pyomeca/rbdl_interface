#ifndef BIORBD_UTILS_ROTOTRANSNODES_H
#define BIORBD_UTILS_ROTOTRANSNODES_H

#include <vector>
#include <memory>
#include "biorbdConfig.h"

namespace biorbd
{

namespace utils
{
class String;
class Matrix;
class RotoTransNode;
}

namespace rigidbody
{
class GeneralizedCoordinates;

///
/// \brief Hold a set of RotoTransNodes
///
class BIORBD_API RotoTransNodes
{
public:
    ///
    /// \brief Construct RT set
    ///
    RotoTransNodes();

    ///
    /// \brief Construct RTs set from another set
    /// \param other The other RTs set
    ///
    RotoTransNodes(
        const biorbd::rigidbody::RotoTransNodes& other);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~RotoTransNodes();

    ///
    /// \brief Deep copy of the RTs data
    /// \return A copy of the RTs data
    ///
    biorbd::rigidbody::RotoTransNodes DeepCopy() const;

    ///
    /// \brief Deep copy the RTs data
    /// \param other The RT data to copy
    ///
    void DeepCopy(
        const biorbd::rigidbody::RotoTransNodes& other);

    ///
    /// \brief Add a new RT to the set
    ///
    void addRT();

    ///
    /// \brief Add a new RT to the set
    /// \param RotoTrans The RotaTrans of the RT
    ///
    void addRT(
        const biorbd::utils::RotoTransNode &RotoTrans);

    ///
    /// \brief Return the number of RTs in the set
    /// \return The number of RTs
    ///
    unsigned int nbRTs() const;

    ///
    /// \brief Return the number of RTs in the set
    /// \return The number of RTs
    ///
    unsigned int size() const;

    ///
    /// \brief Return the names of the RTs
    /// \return The names of the RTs
    ///
    std::vector<biorbd::utils::String> RTsNames();

    ///
    /// \brief Return all the RTs in the local reference of the segment
    /// \return All the RTs in local reference frame
    ///
    const std::vector<biorbd::utils::RotoTransNode>& RTs() const;

    ///
    /// \brief Return all the RTs of a segment
    /// \param segmentName The name of the segment to return the RTs
    /// \return All the RTs of attached to the segment
    ///
    std::vector<biorbd::utils::RotoTransNode> RTs(
        const biorbd::utils::String& segmentName);

    ///
    /// \brief Return the RTs of a specified index
    /// \param idx The index of the RT in the set
    /// \return RT of idx i
    ///
    const biorbd::utils::RotoTransNode& RT(
        unsigned int idx);

    ///
    /// \brief Compute and return all the RTs at the position given by Q
    /// \param Q The generalized coordinates
    /// \param updateKin If the model should be updated
    /// \return All the RTs at the position given by Q
    ///
    std::vector<biorbd::utils::RotoTransNode> RTs(
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        bool updateKin = true);

    ///
    /// \brief Compute and return the RT of index idx at the position given by Q
    /// \param Q The generalized coordinates
    /// \param idx The index of the RT in the set
    /// \param updateKin If the model should be updated
    /// \return The RT of index idx at the position given by Q
    ///
    biorbd::utils::RotoTransNode RT(
        const biorbd::rigidbody::GeneralizedCoordinates&Q,
        unsigned int  idx,
        bool updateKin = true);

    ///
    /// \brief Return all the RTs on a specified segment
    /// \param Q The generalized coordinates
    /// \param idx The index of the segment
    /// \param updateKin If the model should be updated
    /// \return All the RTs on the segment of index idx
    ///
    std::vector<biorbd::utils::RotoTransNode> segmentRTs(
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        unsigned int  idx,
        bool updateKin = true);

    ///
    /// \brief Return the jacobian of the RTs
    /// \param Q The generalized coordinates
    /// \param updateKin If the model should be updated
    /// \return The jacobien of the RTs
    ///
    std::vector<biorbd::utils::Matrix> RTsJacobian(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin = true);

protected:
    std::shared_ptr<std::vector<biorbd::utils::RotoTransNode>>
            m_RTs; ///< All the RTs

};

}
}

#endif // BIORBD_UTILS_ROTOTRANSNODES_H
