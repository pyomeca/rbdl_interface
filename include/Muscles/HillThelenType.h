#ifndef BIORBD_MUSCLES_HILL_THELEN_TYPE_H
#define BIORBD_MUSCLES_HILL_THELEN_TYPE_H

#include "biorbdConfig.h"
#include "Muscles/HillType.h"

namespace biorbd
{
namespace muscles
{
///
/// \brief Muscle of Hill type augmented by Thelen
/// https://simtk-confluence.stanford.edu/display/OpenSim/Thelen+2003+Muscle+Model
///
class BIORBD_API HillThelenType : public biorbd::muscles::HillType
{
public:
    ///
    /// \brief Contruct a Hill-Thelen-type muscle
    ///
    HillThelenType();

    ///
    /// \brief Construct a Hill-Thelen-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    ///
    HillThelenType(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Characteristics& characteristics);

    ///
    /// \brief Construct a Hill-Thelen-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param emg The muscle dynamic state
    ///
    HillThelenType(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Characteristics& characteristics,
        const biorbd::muscles::State& emg);

    ///
    /// \brief Construct a Hill-Thelen-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    ///
    HillThelenType(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Characteristics& characteristics,
        const biorbd::muscles::PathModifiers& pathModifiers);

    ///
    /// \brief Construct a Hill-Thelen-type muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    /// \param emg The dynamic state
    ///
    HillThelenType(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Characteristics& characteristics,
        const biorbd::muscles::PathModifiers& pathModifiers,
        const biorbd::muscles::State& emg);

    ///
    /// \brief Construct a Hill-Thelen-type muscle from another muscle
    /// \param other The other muscle
    ///
    HillThelenType(
        const biorbd::muscles::Muscle& other);

    ///
    /// \brief Construct a Hill-Thelen-type muscle from another muscle
    /// \param other The other muscle (pointer)
    ///
    HillThelenType(
        const std::shared_ptr<biorbd::muscles::Muscle> other);

    ///
    /// \brief Deep copy of a Hill-Thelen-type muscle
    /// \return A deep copy of a Hill-Thelen-type muscle
    ///
    biorbd::muscles::HillThelenType DeepCopy() const;

    ///
    /// \brief Deep copy of a Hill-Thelen-type muscle in a new Hill-Thelen-type muscle
    /// \param other The Hill-Thelen-type muscle to copy
    ///
    void DeepCopy(const biorbd::muscles::HillThelenType& other);

    ///
    /// \brief Compute the Force-Length of the passive element
    ///
    virtual void computeFlPE();

    ///
    /// \brief Compute the Force-Length of the passive element
    /// \param emg EMG data
    ///
    virtual void computeFlCE(const biorbd::muscles::State &emg);

protected:
    ///
    /// \brief Set type to Hill_Thelen
    ///
    virtual void setType();

};

}
}

#endif // BIORBD_MUSCLES_HILL_THELEN_TYPE_H
