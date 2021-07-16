#ifndef BIORBD_UTILS_ENUMS_H
#define BIORBD_UTILS_ENUMS_H

namespace biorbd
{
namespace utils
{

///
/// \brief The available node type
///
enum NODE_TYPE {
    VECTOR3D,
    BONE_POINT,
    ROTOTRANS,
    SEGMENT,
    WRAPPING_OBJECT,
    WRAPPING_HALF_CYLINDER,
    WRAPPING_SPHERE,
    VIA_POINT,
    NO_NODE_TYPE
};

///
/// \brief NODE_TYPE_toStr returns the type name in a string format
/// \param type The type to convert to string
/// \return The name of the type
///
inline const char* NODE_TYPE_toStr(
    biorbd::utils::NODE_TYPE type)
{
    switch (type) {
    case VECTOR3D:
        return "Vector3d";
    case BONE_POINT:
        return "NodeBone";
    case ROTOTRANS:
        return "RotoTrans";
    case SEGMENT:
        return "Segment";
    case WRAPPING_OBJECT:
        return "WrappingObject";
    case WRAPPING_HALF_CYLINDER:
        return "WrappingHalfCylinder";
    case WRAPPING_SPHERE:
        return "WrappinSphere";
    case VIA_POINT:
        return "ViaPoint";
    default:
        return "NoType";
    }
}

}
}

#endif // BIORBD_UTILS_ENUMS_H
