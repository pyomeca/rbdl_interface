#ifndef BIORBD_ACTUATOR_ENUMS_H
#define BIORBD_ACTUATOR_ENUMS_H

namespace biorbd
{
namespace actuator
{

///
/// \brief Types of actuators available
///
enum TYPE {
    CONSTANT,
    LINEAR,
    GAUSS3P,
    GAUSS6P,
    NO_TYPE,
    SIGMOIDGAUSS3P
};

///
/// \brief TYPE_toStr returns the type name in a string format
/// \param type The type to convert to string
/// \return The name of the type
///
inline const char* TYPE_toStr(biorbd::actuator::TYPE type)
{
    switch (type) {
    case CONSTANT:
        return "Constant";
    case LINEAR:
        return "Linear";
    case GAUSS3P:
        return "Gauss3p";
    case GAUSS6P:
        return "Gauss6p";
    case SIGMOIDGAUSS3P:
        return "SigmoidGauss3p";
    default:
        return "NoType";
    }
}


}
}

#endif // BIORBD_ACTUATOR_ENUMS_H
