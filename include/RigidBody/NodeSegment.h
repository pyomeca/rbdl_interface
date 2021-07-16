#ifndef BIORBD_RIGIDBODY_NODE_SEGMENT_H
#define BIORBD_RIGIDBODY_NODE_SEGMENT_H

#include <vector>
#include "biorbdConfig.h"
#include "Utils/Vector3d.h"

namespace biorbd
{
namespace utils
{
class String;
}

namespace rigidbody
{

///
/// \brief A point attached to a segment, generally speaking a skin marker
///
class BIORBD_API NodeSegment : public biorbd::utils::Vector3d
{
public:
    ///
    /// \brief Construct a segment node
    ///
    NodeSegment();

    ///
    /// \brief Construct a segment node
    /// \param x X-Component of the node
    /// \param y Y-Component of the node
    /// \param z Z-Component of the node
    ///
    NodeSegment(
        const biorbd::utils::Scalar& x,
        const biorbd::utils::Scalar& y,
        const biorbd::utils::Scalar& z);

    ///
    /// \brief Construct a segment node from another node
    /// \param other The other node
    ///
    NodeSegment(
        const biorbd::utils::Vector3d& other);

    ///
    /// \brief Construct a segment node
    /// \param x X-Component of the node
    /// \param y Y-Component of the node
    /// \param z Z-Component of the node
    /// \param name The name of the node
    /// \param parentName The name of the parent
    /// \param isTechnical If the node is technical
    /// \param isAnatomical If the node is anatomical
    /// \param axesToRemove The axis to remove
    /// \param parentID The index of the parent segment
    ///
    NodeSegment(
        const biorbd::utils::Scalar& x,
        const biorbd::utils::Scalar& y,
        const biorbd::utils::Scalar& z,
        const biorbd::utils::String& name,
        const biorbd::utils::String& parentName,
        bool isTechnical,
        bool isAnatomical,
        const biorbd::utils::String& axesToRemove,
        int parentID);

    ///
    /// \brief Construct a segment node
    /// \param node The position of the node
    /// \param name The name of the node
    /// \param parentName The name of the parent
    /// \param isTechnical If the node is technical
    /// \param isAnatomical If the node is anatomical
    /// \param axesToRemove The axis to remove
    /// \param parentID The index of the parent segment
    ///
    NodeSegment(
        const biorbd::utils::Vector3d& node,
        const biorbd::utils::String& name,
        const biorbd::utils::String& parentName,
        bool isTechnical,
        bool isAnatomical,
        const biorbd::utils::String& axesToRemove,
        int parentID);

    ///
    /// \brief Deep copy of the segment node
    /// \return A deep copy of the segment node
    ///
    biorbd::rigidbody::NodeSegment DeepCopy() const;

    ///
    /// \brief Deep copy of the segment node
    /// \param other The segment node to copy
    ///
    void DeepCopy(const biorbd::rigidbody::NodeSegment& other);

    // Get and Set

    ///
    /// \brief Return if node is technical
    /// \return If node is technical
    ///
    bool isTechnical() const;

    ///
    /// \brief Return if node is anatomical
    /// \return If node is anatomical
    ///
    bool isAnatomical() const;

    ///
    /// \brief Return the parent index
    /// \return The parent index
    ///
    int parentId() const;

    ///
    /// \brief To remove axis
    /// \return Projected position of the node when removing speficic axes speficied using addAxesToRemove
    ///
    NodeSegment removeAxes() const;

    ///
    /// \brief Check if axis is removed
    /// \return If axis is removed or not
    ///
    bool isAxisRemoved(unsigned int) const;

    ///
    /// \brief Check if axis is kept
    /// \return If axis is kept or not
    ///
    bool isAxisKept(unsigned int) const;

    ///
    /// \brief Add an axis to remove
    /// \param axisNumber The axis number to remove (x = 0, y = 1 and z = 2)
    ///
    void addAxesToRemove(unsigned int axisNumber);

    ///
    /// \brief Add axis to remove
    /// \param axis The name of the axis to remove ("x", "y" or "z")
    ///
    void addAxesToRemove(const biorbd::utils::String& axis);

    ///
    /// \brief Add multiple axes to remove
    /// \param axes The multiples axes numbers to remove (x = 0, y = 1 and z = 2)
    ///
    void addAxesToRemove(const std::vector<unsigned int>& axes);

    ///
    /// \brief Add multiple axes to remove
    /// \param axes The multiples axes names to remove ("x", "y" or "z")
    ///
    void addAxesToRemove(const std::vector<biorbd::utils::String>& axes);

    ///
    /// \brief Return the axes to removed
    /// \return The axes to removed
    ///
    biorbd::utils::String axesToRemove();

    ///
    /// \brief Return the number of axes to remove
    /// \return The number of axes to remove
    ///
    int nbAxesToRemove() const;

#ifndef SWIG
#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Allows for operator= with other Vector3d
    /// \param other The other vector
    ///
    template<typename OtherDerived>
    biorbd::rigidbody::NodeSegment & operator=(const Eigen::MatrixBase
            <OtherDerived>& other)
    {
        this->biorbd::utils::Vector3d::operator=(other);
        return *this;
    }
#endif
#endif

protected:
    ///
    /// \brief Set the type of the segment node
    ///
    void setType();

    std::shared_ptr<std::vector<bool>> m_axesRemoved; ///< The axes to remove
    std::shared_ptr<int>
    m_nbAxesToRemove; ///< Removed one of multiple axes (1 axis : project on a plan, 2 axes : project on the 3rd axis, 3 axes : return the position of the parent reference)
    std::shared_ptr<bool> m_technical; ///< If a marker is a technical marker
    std::shared_ptr<bool> m_anatomical; ///< It marker is a anatomical marker
    std::shared_ptr<int> m_id; ///< The parent identification

};

}
}

#endif // BIORBD_RIGIDBODY_NODE_SEGMENT_H
