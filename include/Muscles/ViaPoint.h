#ifndef BIORBD_MUSCLES_VIAPOINT_H
#define BIORBD_MUSCLES_VIAPOINT_H

#include "biorbdConfig.h"
#include "Utils/Vector3d.h"

namespace biorbd
{
namespace utils
{
class String;
}

namespace muscles
{
///
/// \brief Via point of a muscle
///
class BIORBD_API ViaPoint : public biorbd::utils::Vector3d
{
public:
    ///
    /// \brief Contruct ViaPoint
    ///
    ViaPoint();

    ///
    /// \brief Construct ViaPoint
    /// \param x X-Component of the ViaPoint
    /// \param y Y-Component of the ViaPoint
    /// \param z Z-Component of the ViaPoint
    ///
    ViaPoint(
        const biorbd::utils::Scalar& x,
        const biorbd::utils::Scalar& y,
        const biorbd::utils::Scalar& z);

    ///
    /// \brief Construct ViaPoint
    /// \param x X-Component of the ViaPoint
    /// \param y Y-Component of the ViaPoint
    /// \param z Z-Component of the ViaPoint
    /// \param name The name of the via point
    /// \param parentName The name of the parent segment
    ///
    ViaPoint(
        const biorbd::utils::Scalar& x,
        const biorbd::utils::Scalar& y,
        const biorbd::utils::Scalar& z,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName);

    ///
    /// \brief Construct ViaPoint from a vector
    /// \param other The vector
    ///
    ViaPoint(
        const biorbd::utils::Vector3d& other);

    ///
    /// \brief Construct ViaPoint from another ViaPoint
    /// \param other The other ViaPoint
    ///
    ViaPoint(
        const biorbd::muscles::ViaPoint& other);

    ///
    /// \brief Deep copy of a ViaPoint
    /// \return A deep copy of a ViaPoint
    ///
    biorbd::muscles::ViaPoint DeepCopy() const;

    ///
    /// \brief Deep copy of a ViaPoint into another ViaPoint
    /// \param other The ViaPoint to copy
    ///
    void DeepCopy(const biorbd::muscles::ViaPoint& other);

    ///
    /// \brief To be able to use operator "=" when assigning a ViaPoint to a 3D node
    /// \param other The 3D node to assign to ViaPoint
    ///
    template<typename OtherDerived>
    biorbd::muscles::ViaPoint& operator=(const biorbd::utils::Vector3d& other)
    {
        this->biorbd::utils::Vector3d::operator=(other);
        return *this;
    }
};

}
}

#endif // BIORBD_MUSCLES_VIAPOINT_H
