#ifndef BIORBD_MUSCLES_HILL_THELEN_TYPE_FATIGABLE_H
#define BIORBD_MUSCLES_HILL_THELEN_TYPE_FATIGABLE_H

#include "biorbdConfig.h"

#include "Muscles/HillThelenType.h"
#include "Muscles/FatigueModel.h"

namespace biorbd
{
namespace utils
{
class String;
}

namespace muscles
{

///
/// \brief Add a fatigue model to the HillThelen type
/// Note that useful defaults values for the FatigueParameters characteristics are:
/// fatigueRate = 0.01
/// recoveryRate = 0.002
/// developFactor = 10
/// recoverFactor = 10
///
class BIORBD_API HillThelenTypeFatigable : public
    biorbd::muscles::HillThelenType, public biorbd::muscles::FatigueModel
{
public:
    ///
    /// \brief Contruct a Hill-Thelen-type fatigable muscle
    ///
    HillThelenTypeFatigable();

    ///
    /// \brief Construct a Hill-Thelen-type fatigable muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param dynamicFatigueType The muscle dynamic fatigue type
    ///
    HillThelenTypeFatigable(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Characteristics& characteristics,
        biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType =
            biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    ///
    /// \brief Construct a Hill-Thelen-type fatigable muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param emg The muscle dynamic state
    /// \param dynamicFatigueType The muscle dynamic fatigue model
    ///
    HillThelenTypeFatigable(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Characteristics& characteristics,
        const biorbd::muscles::State& emg,
        biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType =
            biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    ///
    /// \brief Construct a Hill-Thelen-type fatigable muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    /// \param dynamicFatigueType The muscle dynamic fatigue model
    ///
    HillThelenTypeFatigable(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Characteristics& characteristics,
        const biorbd::muscles::PathModifiers& pathModifiers,
        biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType =
            biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    ///
    /// \brief Construct a Hill-Thelen-type fatigable muscle
    /// \param name The muscle name
    /// \param geometry The muscle geometry
    /// \param characteristics The muscle characteristics
    /// \param pathModifiers The set of path modifiers
    /// \param emg The muscle dynamic state
    /// \param dynamicFatigueType The muscle dynamic fatigue model
    ///
    HillThelenTypeFatigable(
        const biorbd::utils::String& name,
        const biorbd::muscles::Geometry& geometry,
        const biorbd::muscles::Characteristics& characteristics,
        const biorbd::muscles::PathModifiers& pathModifiers,
        const biorbd::muscles::State& emg,
        biorbd::muscles::STATE_FATIGUE_TYPE dynamicFatigueType =
            biorbd::muscles::STATE_FATIGUE_TYPE::SIMPLE_STATE_FATIGUE);

    ///
    /// \brief Construct a Hill-Thelen-type fatigable muscle from another muscle
    /// \param other The other muscle
    ///
    HillThelenTypeFatigable(
        const biorbd::muscles::Muscle& other);

    ///
    /// \brief Construct a Hill-Thelen-type muscle from another muscle
    /// \param other The other muscle (pointer)
    ///
    HillThelenTypeFatigable(
        const std::shared_ptr<biorbd::muscles::Muscle> other);

    ///
    /// \brief Deep copy of a Hill-Thelen-type fatigable muscle
    /// \return A deep copy of a Hill-Thelen-type fatigable muscle
    ///
    biorbd::muscles::HillThelenTypeFatigable DeepCopy() const;


    ///
    /// \brief Deep copy of a Hill-Thelen-type fatigable muscle in a new Hill-Thelen-type fatigable muscle
    /// \param other The Hill-Thelen-type fatigable muscle to copy
    ///
    void DeepCopy(const biorbd::muscles::HillThelenTypeFatigable& other);

    ///
    /// \brief Compute the Force-Length of the contractile element
    /// \param emg EMG data
    ///
    virtual void computeFlCE(const biorbd::muscles::State &emg);

protected:
    ///
    /// \brief Set type to Hill_Thelen_Fatigable
    ///
    virtual void setType();

};

}
}

#endif // BIORBD_MUSCLES_HILL_THELEN_TYPE_FATIGABLE_H
