#ifndef BIORBD_MUSCLES_COMPOUND_H
#define BIORBD_MUSCLES_COMPOUND_H

#include <memory>
#include <vector>
#include "biorbdConfig.h"
#include "Muscles/MusclesEnums.h"
#include "Utils/Scalar.h"

namespace biorbd
{
namespace utils
{
class String;
class Vector3d;
}

namespace rigidbody
{
class Joints;
class GeneralizedCoordinates;
class GeneralizedVelocity;
}

namespace muscles
{
class Characteristics;
class PathModifiers;
class State;
///
/// \brief Class compound is a very generic definition of what a muscle is. It should be the base class of every muscles
///
class BIORBD_API Compound
{
public:
    ///
    /// \brief Construct muscle compound
    ///
    Compound();

    ///
    /// \brief Construct compound
    /// \param name Name of the compound
    ///
    Compound(
        const biorbd::utils::String &name);

    ///
    /// \brief Construct compound
    /// \param name Name of the compound
    /// \param pathModifiers The set of path modifiers
    ///
    Compound(
        const biorbd::utils::String& name,
        const biorbd::muscles::PathModifiers& pathModifiers);
    ///
    /// \brief Construct compound from another muscle
    /// \param other The muscle to shallow copy
    ///
    Compound(
        const biorbd::muscles::Compound& other);

    ///
    /// \brief Construct compound from another muscle
    /// \param other The muscle to shallow copy
    ///
    Compound(
        const std::shared_ptr<biorbd::muscles::Compound> other);

    ///
    /// \brief Destroy class properly
    ///
    virtual ~Compound();

    ///
    /// \brief Deep copy of a compound
    /// \param other Compound to copy
    ///
    void DeepCopy(
        const biorbd::muscles::Compound& other);

    ///
    /// \brief Set the name of a muscle
    /// \param name Name of the muscle
    ///
    void setName(const biorbd::utils::String& name);

    ///
    /// \brief Return the name of the muscle
    /// \return The name of the muscle
    ///
    const biorbd::utils::String& name() const;


    ///
    /// \brief Return the type of the muscle
    /// \return The type of the muscle
    ///
    biorbd::muscles::MUSCLE_TYPE type() const;

    // Wrapping object
    ///
    /// \brief Return the path modifier
    /// \return The path modifier
    ///
    const biorbd::muscles::PathModifiers& pathModifier();

    ///
    /// \brief Add a path modifier object
    /// \param wrap Position of the object
    ///
    void addPathObject(biorbd::utils::Vector3d& wrap);

    ///
    /// \brief Return the last computed muscle force norm
    /// \return The last computed muscle force norm
    ///
    virtual const biorbd::utils::Scalar& force();

    ///
    /// \brief Computes and returns the forces norm from the EMG
    /// \param emg EMG data
    /// \return The computed forces from the EMG
    ///
    virtual const biorbd::utils::Scalar& force(
        const biorbd::muscles::State& emg) = 0;

    ///
    /// \brief Return the computed force norm from EMG
    /// \param model The joints model
    /// \param Q The generalized coordinates of the model
    /// \param Qdot The generalized velocities of the model
    /// \param emg EMG data
    /// \param updateKin Update kinematics (0: don't update, 1:only muscles, [2: both kinematics and muscles])
    /// \return The computed force from EMG
    ///
    virtual const biorbd::utils::Scalar& force(
        biorbd::rigidbody::Joints& model,
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::rigidbody::GeneralizedVelocity& Qdot,
        const biorbd::muscles::State& emg,
        int updateKin = 2) = 0;

    ///
    /// \brief Return the computed force norm from EMG
    /// \param model The joints model
    /// \param Q The generalized coordinates of the model
    /// \param emg EMG data
    /// \param updateKin Update kinematics (0: don't update, 1:only muscles, [2: both kinematics and muscles])
    /// \return The computed force from EMG
    ///
    virtual const biorbd::utils::Scalar& force(
        biorbd::rigidbody::Joints& model,
        const biorbd::rigidbody::GeneralizedCoordinates& Q,
        const biorbd::muscles::State& emg,
        int updateKin = 2) = 0;

protected:
    std::shared_ptr<biorbd::utils::String> m_name; ///< The name of the muscle
    std::shared_ptr<biorbd::muscles::MUSCLE_TYPE> m_type; ///< The type of muscle
    std::shared_ptr<biorbd::muscles::PathModifiers>
    m_pathChanger; ///< The set of path modifiers
    std::shared_ptr<biorbd::utils::Scalar> m_force; ///< The last computed force

    ///
    /// \brief Set the type of muscle
    ///
    virtual void setType()=0;

};

}
}

#endif // BIORBD_MUSCLES_COMPOUND_H
