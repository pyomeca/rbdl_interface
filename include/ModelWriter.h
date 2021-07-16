#ifndef BIORBD_UTILS_WRITER_H
#define BIORBD_UTILS_WRITER_H

#include "biorbdConfig.h"
namespace biorbd
{
class Model;

namespace utils
{
class Path;
}

///
/// \brief Main class for writer
///
class BIORBD_API Writer
{
public:
#ifndef BIORBD_USE_CASADI_MATH
    ///
    /// \brief Writes the model on the computer
    /// \param model The model to write
    /// \param pathToWrite The path to write
    ///
    static void writeModel(
        biorbd::Model &model,
        const biorbd::utils::Path& pathToWrite);
#endif
};

}

#endif // BIORBD_UTILS_WRITER_H
