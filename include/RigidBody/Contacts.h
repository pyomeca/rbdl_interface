#ifndef BIORBD_RIGIDBODY_CONTACTS_H
#define BIORBD_RIGIDBODY_CONTACTS_H

#include <vector>
#include <memory>
#include <rbdl/Constraints.h>
#include "biorbdConfig.h"

namespace biorbd
{
namespace utils
{
class RotoTrans;
class Vector3d;
class Vector;
class String;
class SpatialVector;
}
}

namespace biorbd
{
namespace rigidbody
{
class Joints;
class GeneralizedCoordinates;

///
/// \brief Class Contacts
///
#ifdef SWIG
class BIORBD_API Contacts
#else
class BIORBD_API Contacts : public RigidBodyDynamics::ConstraintSet
#endif
{
public:
    ///
    /// \brief Construct contacts
    ///
    Contacts();

    ///
    /// \brief Deep copy of contacts
    /// \return Copy of contacts
    ///
    biorbd::rigidbody::Contacts DeepCopy() const;

    ///
    /// \brief Deep copy of contacts
    /// \param other The contacts to copy
    ///
    void DeepCopy(
        const biorbd::rigidbody::Contacts& other);

    ///
    /// \brief Add a constraint to the constraint set
    /// \param body_id The body which is affected directly by the constraint
    /// \param body_point The point that is constrained relative to the contact body
    /// \param world_normal The normal along the constraint acts (in base coordinates)
    /// \param name A human readable name
    /// \param acc The acceleration of the contact along the normal
    ///
    unsigned int AddConstraint(
        unsigned int body_id,
        const biorbd::utils::Vector3d &body_point,
        const biorbd::utils::Vector3d &world_normal,
        const biorbd::utils::String& name,
        double acc = 0);

    ///
    /// \brief Add a constraint to the constraint set
    /// \param body_id The body which is affected directly by the constraint
    /// \param body_point The point that is constrained relative to the contact body
    /// \param axis The axis along which the constraint acts
    /// \param name A human readable name
    /// \param acc The acceleration of the contact along the normal
    ///

    unsigned int AddConstraint(
        unsigned int body_id,
        const biorbd::utils::Vector3d &body_point,
        const biorbd::utils::String& axis,
        const biorbd::utils::String& name,
        double acc = 0);

    ///
    /// \brief Add a loop constraint to the constraint set
    /// \param body_id_predecessor The identifier of the predecessor body
    /// \param body_id_successor The identifier of the successor body
    /// \param X_predecessor A spatial transform localizing the constrained frames on the predecessor body, expressed with respect to the predecessor body frame
    /// \param X_successor A spatial transform localizing the constrained frames on the successor body, expressed with respect to the successor body frame
    /// \param axis A spatial vector indicating the axis along which the constraint acts
    /// \param name A human readable name
    /// \param enableStabilization Whether stabilization should be enabled or not
    /// \param stabilizationParam The value used for stabilization
    ///
    unsigned int AddLoopConstraint(
        unsigned int body_id_predecessor,
        unsigned int body_id_successor,
        const biorbd::utils::RotoTrans& X_predecessor,
        const biorbd::utils::RotoTrans& X_successor,
        const biorbd::utils::SpatialVector& axis,
        const biorbd::utils::String& name,
        bool enableStabilization = false,
        double stabilizationParam = 0.1);

    ///
    /// \brief Destroy the class properly
    ///
    virtual ~Contacts();

    ///
    /// \brief Get constraints
    /// \return The constraints
    ///
    Contacts &getConstraints();

    ///
    /// \brief Check if there are contacts
    /// \return The presence of contacts
    ///
    bool hasContacts() const;

    ///
    /// \brief Return the number of contacts
    /// \return The number of contacts
    ///
    unsigned int nbContacts() const;

    ///
    /// \brief Return the name of the all contacts
    /// \return The name of the contacts
    ///
    std::vector<biorbd::utils::String> contactNames();

    ///
    /// \brief Return the name of the contact of a specified axis
    /// \param i The axis
    /// \return The name of the contact of a specified axis
    ///
    biorbd::utils::String contactName(unsigned int i);

    ///
    /// \brief Return the contraints position in the global reference
    /// \param Q The generalized coordinates of the joints
    /// \param updateKin Whether the kinematics of the model should be updated from Q
    /// \return The contraints positions in the global reference
    ///
    std::vector<biorbd::utils::Vector3d> constraintsInGlobal(
        const biorbd::rigidbody::GeneralizedCoordinates &Q,
        bool updateKin);

    ///
    /// \brief Return the force acting on the contraint
    /// \return The force acting on the contraint
    ///
    biorbd::utils::Vector getForce() const;

protected:
    std::shared_ptr<unsigned int> m_nbreConstraint; ///< Number of constraints
    std::shared_ptr<bool> m_isBinded; ///< If the model is ready

};

}
}

#endif // BIORBD_RIGIDBODY_CONTACTS_H
