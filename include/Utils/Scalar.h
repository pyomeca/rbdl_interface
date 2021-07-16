#ifndef BIORBD_UTILS_SCALAR_H
#define BIORBD_UTILS_SCALAR_H

#include "biorbdConfig.h"
#include "rbdl/rbdl_math.h"

namespace biorbd
{
namespace utils
{


#ifdef BIORBD_USE_CASADI_MATH

///
/// \brief Wrapper of scalar
///
#ifdef SWIG
class Scalar
{
#else
class Scalar : public RigidBodyDynamics::Math::Scalar
{
#endif

public:
    ///
    /// \brief Constructor for a casadi::MX scalar
    ///
    Scalar();

    ///
    /// \brief Constructor for a casadi::MX scalar
    /// \param val The value to copy
    ///
    Scalar(
        double val);

    ///
    /// \brief Constructor for a casadi::MX scalar
    /// \param val The value to copy. A test is performed to ensure val is 1x1
    ///
    Scalar(
        const casadi::MX& val);

};

#else

#ifdef SWIG
    typedef double Scalar;
#else
    typedef RigidBodyDynamics::Math::Scalar Scalar;
#endif

#endif

}
}

#endif // BIORBD_UTILS_SCALAR_H
