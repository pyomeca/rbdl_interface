#ifndef BIORBD_RIGIDBODY_IMU_H
#define BIORBD_RIGIDBODY_IMU_H

#include <memory>
#include "biorbdConfig.h"
#include "Utils/RotoTransNode.h"

namespace biorbd
{
namespace utils
{
class String;
}

namespace rigidbody
{

///
/// \brief Class IMU
///
#ifdef SWIG
class BIORBD_API IMU
#else
class BIORBD_API IMU : public biorbd::utils::RotoTransNode
#endif
{
public:

    ///
    /// \brief Construct inertial measurement unit
    /// \param isTechnical True if the IMU is a technical IMU
    /// \param isAnatomical True if the IMU is an anatomical IMU
    ///
    IMU(
        bool isTechnical = true,
        bool isAnatomical = true);

    ///
    /// \brief Construct inertial measurement unit data
    /// \param RotoTrans The RotoTrans matrix attaching the IMU
    /// \param isTechnical True if the IMU is a technical IMU
    /// \param isAnatomical True if the IMU is an anatomical IMU
    ///
    IMU(
        const biorbd::utils::RotoTransNode& RotoTrans,
        bool isTechnical = true,
        bool isAnatomical = true);

#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Construct inertial measurement unit data from a Matrix4d
    /// \param other The other matrix
    ///
    template<typename OtherDerived> IMU(
        const Eigen::MatrixBase<OtherDerived>& other) :
        biorbd::utils::RotoTransNode(other) {}
#endif
#ifdef BIORBD_USE_CASADI_MATH
    IMU(
        const biorbd::rigidbody::IMU& imu);

    biorbd::rigidbody::IMU operator*(const biorbd::rigidbody::IMU& other) const;
#endif


    ///
    /// \brief Deep copy of the IMU data
    /// \return Copy of the IMU data
    ///
    biorbd::rigidbody::IMU DeepCopy() const;

    ///
    /// \brief Deep copy if the IMU data
    /// \param other The IMU to copy
    ///
    void DeepCopy(const biorbd::rigidbody::IMU& other);

    ///
    /// \brief Return if the IMU is technical
    /// \return If the IMU is technical
    ///
    bool isTechnical() const;

    ///
    /// \brief Return if the IMU is anatomical
    /// \return If the IMU is anatomical
    ///
    bool isAnatomical() const;

#ifndef SWIG
#ifdef BIORBD_USE_EIGEN3_MATH
    ///
    /// \brief Allows for operator= to be used
    /// \param other
    /// \return The current IMU
    ///
    template<typename OtherDerived>
    biorbd::rigidbody::IMU& operator=(const Eigen::MatrixBase <OtherDerived>& other)
    {
        this->biorbd::utils::RotoTransNode::operator=(other);
        return *this;
    }
#endif
#endif

protected:
    std::shared_ptr<bool> m_technical; ///< If a IMU is a technical IMU
    std::shared_ptr<bool> m_anatomical; ///< It IMU is a anatomical IMU

};

}
}

#endif // BIORBD_RIGIDBODY_IMU_H
