#ifndef BIORBD_MUSCLES_WRAPPING_OBJECT_H
#define BIORBD_MUSCLES_WRAPPING_OBJECT_H

#include "biorbdConfig.h"
#include "Utils/Scalar.h"
#include "Utils/Vector3d.h"

namespace biorbd
{
namespace utils
{
class String;
}

namespace rigidbody
{
class Joints;
class GeneralizedCoordinates;
}

namespace muscles
{
///
/// \brief Base class for the wrapping objects
///
class BIORBD_API WrappingObject : public biorbd::utils::Vector3d
{
public:
    ///
    /// \brief Construct a wrapping object
    ///
    WrappingObject();

    ///
    /// \brief Construct a wrapping object
    /// \param x X-Component of the wrapping object
    /// \param y Y-Component of the wrapping object
    /// \param z Z-Component of the wrapping object
    ///
    WrappingObject(
        const biorbd::utils::Scalar& x,
        const biorbd::utils::Scalar& y,
        const biorbd::utils::Scalar& z);
    ///
    /// \brief Construct a wrapping object
    /// \param x X-Component of the wrapping object
    /// \param y Y-Component of the wrapping object
    /// \param z Z-Component of the wrapping object
    /// \param name Name of the wrapping object
    /// \param parentName Name of the parent segment
    ///
    WrappingObject(
        const biorbd::utils::Scalar& x,
        const biorbd::utils::Scalar& y,
        const biorbd::utils::Scalar& z,
        const biorbd::utils::String &name,
        const biorbd::utils::String &parentName);

    ///
    /// \brief Construct a wrapping object
    /// \param other Eigen vector
    ///
    WrappingObject(
        const biorbd::utils::Vector3d& other);

    ///
    /// \brief Construct a wrapping object
    /// \param other Eigen vector
    /// \param name Name of the wrapping object
    /// \param parentName Name of the parent segment
    ///
    WrappingObject(
        const biorbd::utils::Vector3d& other,
        const biorbd::utils::String& name,
        const biorbd::utils::String& parentName);

    ///
    /// \brief Deep copy of the wrapping ibject in another wrapping object
    /// \param other The wrapping object to copy
    ///
    void DeepCopy(
        const biorbd::muscles::WrappingObject& other);

    ///
    /// \brief From the position of the wrapping object, return the 2 locations where the muscle leaves the wrapping object
    /// \param rt RotoTrans matrix of the wrapping object
    /// \param p1_bone 1st position of the muscle node
    /// \param p2_bone 2n position of the muscle node
    /// \param p1 The 1st position on the wrapping object the muscle leave
    /// \param p2 The 2nd position on the wrapping object the muscle leave
    /// \param muscleLength Length of the muscle (ignored if no value is provided)
    ///
    virtual void wrapPoints(
        const biorbd::utils::RotoTrans& rt,
        const biorbd::utils::Vector3d& p1_bone,
        const biorbd::utils::Vector3d& p2_bone,
        biorbd::utils::Vector3d& p1,
        biorbd::utils::Vector3d& p2,
        biorbd::utils::Scalar* muscleLength = nullptr) = 0
                ; // Premier et dernier points musculaire

    ///
    /// \brief From the position of the wrapping object, return the 2 locations where the muscle leaves the wrapping object
    /// \param model The joint model
    /// \param Q The generalized coordinates
    /// \param p1_bone 1st position of the muscle node
    /// \param p2_bone 2n position of the muscle node
    /// \param p1 The 1st position on the wrapping object the muscle leave
    /// \param p2 The 2nd position on the wrapping object the muscle leave
    /// \param muscleLength Length of the muscle (ignored if no value is provided)
    ///
    virtual void wrapPoints(
        biorbd::rigidbody::Joints& model,
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::utils::Vector3d& p1_bone,
        const biorbd::utils::Vector3d& p2_bone,
        biorbd::utils::Vector3d& p1,
        biorbd::utils::Vector3d& p2,
        biorbd::utils::Scalar* muscleLength = nullptr) =
            0; // Premier et dernier points musculaire

    ///
    /// \brief Returns the previously computed 2 locations where the muscle leaves the wrapping object
    /// \param p1 The 1st position on the wrapping object the muscle leave
    /// \param p2 The 2nd position on the wrapping object the muscle leave
    /// \param muscleLength Length of the muscle (ignored if no value is provided)
    ///
    virtual void wrapPoints(
        biorbd::utils::Vector3d& p1,
        biorbd::utils::Vector3d& p2,
        biorbd::utils::Scalar* muscleLength = nullptr) =
            0; // Assume un appel déja faits

    ///
    /// \brief Return the RotoTrans matrix of the wrapping object
    /// \param model The joint model
    /// \param Q The generalized coordinates
    /// \param updateKin If the kinematics should be computed
    /// \return The RotoTrans matrix of the wrapping object
    ///
    virtual const biorbd::utils::RotoTrans& RT(
        biorbd::rigidbody::Joints &model,
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        bool updateKin = true) = 0;

    ///
    /// \brief Return the RotoTrans matrix of the wrapping object
    /// \return The RotoTrans matrix of the wrapping object
    ///
    const biorbd::utils::RotoTrans& RT() const;

#ifndef SWIG
    ///
    /// \brief To be able to use the equal "=" operator to define wrapping object
    /// \param other The 3d node to define the wrapping object
    biorbd::muscles::WrappingObject& operator=(const biorbd::utils::Vector3d& other)
    {
        this->biorbd::utils::Vector3d::operator=(other);
        return *this;
    }
#endif
protected:
    std::shared_ptr<biorbd::utils::RotoTrans>
    m_RT; ///< RotoTrans matrix of the wrapping object
};

}
}

#endif // BIORBD_MUSCLES_WRAPPING_OBJECT_H
