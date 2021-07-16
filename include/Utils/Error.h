#ifndef BIORBD_UTILS_ERROR_H
#define BIORBD_UTILS_ERROR_H

#include "biorbdConfig.h"
#include "Utils/String.h"

namespace biorbd
{
namespace utils
{
class String;

///
/// \brief Raise error or warning while biorbding
///
class BIORBD_API Error
{
public:
    ///
    /// \brief Throw an error message
    /// \param message The error message to display
    ///
    [[noreturn]] static void raise(
        const biorbd::utils::String &message);

    ///
    /// \brief Assert that raises the error message if false
    /// \param cond The condition to assert
    /// \param message The error message to display in case of failing
    ///
    static void check(
        bool cond,
        const biorbd::utils::String &message);

    ///
    /// \brief Non-blocking assert that displays the error message if false
    /// \param cond The condition to assert
    /// \param message The warning message to display in case of failing
    ///
    static void warning(
        bool cond,
        const biorbd::utils::String &message);
};

}
}

#endif // BIORBD_UTILS_ERROR_H


