#ifndef BIORBD_MODEL_H
#define BIORBD_MODEL_H

#include "biorbdConfig.h"
#include "Utils/Path.h"
#include "RigidBody/RotoTransNodes.h"
#include "RigidBody/Joints.h"
#include "RigidBody/Markers.h"
#include "RigidBody/Contacts.h"
#include "RigidBody/IMUs.h"
#ifdef MODULE_ACTUATORS
    #include "Actuators/Actuators.h"
#endif
#ifdef MODULE_MUSCLES
    #include "Muscles/Muscles.h"
#endif

///
/// \mainpage Documentation of biorbd
///
/// \section intro_sec Introduction
///
/// This is the document for the library biorbd
/// (<a href="http://github.com/pyomeca/biorbd">http://github.com/pyomeca/biorbd</a>).
/// The main goal of this library is to provide biomechanics tools for simulation
/// and modeling.
///
/// biorbd is a library to analyze biomechanical data. It provides several useful
/// functions for the direct and inverse flow including rigid body (based on
/// Feathestone equations implemented in RBDL) and muscle elements.
/// Biomechanical data are often analyzed using similar flow, that is
/// inverse or direct. biorbd implements these common analyses providing
/// high-level and easy to use Python and MATLAB interfaces of an
/// efficient C++ implementation.
///
/// This documentation was automatically generated for the "ShowMeWhatYouveGot"
/// Release 1.3.3 on the 15th of June, 2020.
///
/// \section install_sec Installation
///
/// To install biorbd, please refer to the README.md file accessible via the
/// github repository or by following this
/// <a href="md__home_pariterre_programmation_biorbd_README.html">link</a>.
///
/// \section contact_sec Contact
///
/// If you have any questions, comments or suggestions for future development,
/// you are very welcomed to send me an email at
/// <a href="mailto:pariterre@gmail.com">pariterre@gmail.com</a>.
///
/// \section conclusion_sec Conclusion
///
/// Enjoy biorbding!
///

///
/// \brief Returns the current version of biorbd
/// \return The current version of biorbd
///
biorbd::utils::String getVersion();

namespace biorbd
{
///
/// \brief The actual musculoskeletal model that holds everything in biorbd
///
class BIORBD_API Model :
    public biorbd::rigidbody::Joints
    ,public biorbd::rigidbody::Markers
    ,public biorbd::rigidbody::IMUs
    ,public biorbd::rigidbody::RotoTransNodes
    ,public biorbd::rigidbody::Contacts
#ifdef MODULE_ACTUATORS
    ,public biorbd::actuator::Actuators
#endif
#ifdef MODULE_MUSCLES
    ,public biorbd::muscles::Muscles
#endif
{
public:
    ///
    /// \brief Construct an empty model that can be manually filled
    ///
    Model();

    ///
    /// \brief Construct a model from a bioMod file
    /// \param path The path of the file
    ///
    Model(
        const biorbd::utils::Path& path);

private:
    std::shared_ptr<biorbd::utils::Path> m_path;
public:
    ///
    /// \brief Returns the path of .bioMod file used to load the model. If no file was used, it remains empty
    /// \return The path of .bioMod file used to load the model
    ///
    biorbd::utils::Path path() const;
};

}

#endif // BIORBD_MODEL_H
