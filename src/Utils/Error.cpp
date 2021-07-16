#define BIORBD_API_EXPORTS
#include "Utils/Error.h"

void biorbd::utils::Error::raise(
    const biorbd::utils::String &message)
{
    throw std::runtime_error(message);
}

void biorbd::utils::Error::check(
    bool cond,
    const biorbd::utils::String& message)
{
    if (!cond) {
        throw std::runtime_error(message);
    }

}

void biorbd::utils::Error::warning(
    bool cond,
    const biorbd::utils::String& message)
{
    if (!cond) {
        std::cout << "Warning: " << message << std::endl;
    }
}
