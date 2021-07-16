#define BIORBD_API_EXPORTS
#include "Utils/Rotation.h"

#include "Utils/Error.h"
#include "Utils/Vector3d.h"
#include "Utils/String.h"
#include "Utils/Vector.h"
#include "Utils/Matrix.h"

#include "RigidBody/NodeSegment.h"

biorbd::utils::Rotation::Rotation(
    const RigidBodyDynamics::Math::Matrix3d& matrix) :
    RigidBodyDynamics::Math::Matrix3d(matrix)
{
    checkUnitary();
}

#ifdef BIORBD_USE_CASADI_MATH

biorbd::utils::Rotation::Rotation(
    const RigidBodyDynamics::Math::MatrixNd &m) :
    RigidBodyDynamics::Math::Matrix3d(m)
{

}

#endif

biorbd::utils::Rotation::Rotation(
    const biorbd::utils::Scalar& v00, const biorbd::utils::Scalar& v01,
    const biorbd::utils::Scalar& v02,
    const biorbd::utils::Scalar& v10, const biorbd::utils::Scalar& v11,
    const biorbd::utils::Scalar& v12,
    const biorbd::utils::Scalar& v20, const biorbd::utils::Scalar& v21,
    const biorbd::utils::Scalar& v22) :
    RigidBodyDynamics::Math::Matrix3d (v00, v01, v02, v10, v11, v12, v20, v21, v22)
{

}

biorbd::utils::Rotation::Rotation(
    const biorbd::utils::Vector& rotation,
    const biorbd::utils::String& rotationSequence) :
    RigidBodyDynamics::Math::Matrix3d(fromEulerAngles(rotation, rotationSequence))
{

}

biorbd::utils::Rotation::Rotation(
    const RigidBodyDynamics::Math::SpatialTransform &st) :
    RigidBodyDynamics::Math::Matrix3d(st.E)
{

}

biorbd::utils::Vector3d biorbd::utils::Rotation::axe(unsigned int idx) const
{
    biorbd::utils::Error::check(idx<=2, "Axis must be between 0 and 2 included");
    return this->block(0,idx, 3, 1);
}

biorbd::utils::Rotation biorbd::utils::Rotation::fromSpatialTransform(
    const RigidBodyDynamics::Math::SpatialTransform& st)
{
    return st.E;
}

biorbd::utils::Rotation biorbd::utils::Rotation::fromEulerAngles(
    const biorbd::utils::Vector &rot,
    const biorbd::utils::String& seq)
{
    // Check for size consistency
    biorbd::utils::Error::check(
        seq.length() == static_cast<unsigned int>(rot.rows()),
        "Rotation and sequence of rotation must be the same length");

    biorbd::utils::Rotation out;
    out.setIdentity();
    // Set the actual rotation matrix to this
    RigidBodyDynamics::Math::Matrix3d tp;
    for (unsigned int i=0; i<seq.length(); ++i) {
        biorbd::utils::Scalar cosVi(std::cos(rot[i]));
        biorbd::utils::Scalar sinVi(std::sin(rot[i]));
        if (seq.tolower()[i] == 'x')
            tp = RigidBodyDynamics::Math::Matrix3d(1,     0,      0,
                                                   0, cosVi, -sinVi,
                                                   0, sinVi,  cosVi);

        else if (seq.tolower()[i] == 'y')
            tp = RigidBodyDynamics::Math::Matrix3d(cosVi, 0, sinVi,
                                                   0, 1,     0,
                                                   -sinVi, 0, cosVi);

        else if (seq.tolower()[i] == 'z')
            tp = RigidBodyDynamics::Math::Matrix3d(cosVi, -sinVi,  0,
                                                   sinVi,  cosVi,  0,
                                                   0,      0,  1);
        else {
            biorbd::utils::Error::raise("Rotation sequence not recognized");
        }

        out.block(0,0,3,3) = out.block(0,0,3,3) * tp;
    }
    return out;
}

biorbd::utils::Matrix biorbd::utils::Rotation::fromMarkersNonNormalized(
    const std::pair<biorbd::rigidbody::NodeSegment, biorbd::rigidbody::NodeSegment>
    &axis1markers,
    const std::pair<biorbd::rigidbody::NodeSegment, biorbd::rigidbody::NodeSegment>
    &axis2markers,
    const std::pair<biorbd::utils::String, biorbd::utils::String>& axesNames,
    const biorbd::utils::String &axisToRecalculate)
{
    if (!axesNames.first.compare("") || !axesNames.second.compare("")) {
        biorbd::utils::Error::raise("axesNames must be defined with a pair of \"x\", \"y\" or \"z\"");
    }

    // Figure out where to put the axes
    std::vector<unsigned int> map(3);
    std::vector<unsigned int> toMultiply(2);
    if (!axesNames.first.tolower().compare("x")) {
        map[0] = 0;
        if (!axesNames.second.tolower().compare("y")) {
            map[1] = 1;
            map[2] = 2;
            toMultiply[0] = 0;
            toMultiply[1] = 1;
        } else {
            map[1] = 2;
            map[2] = 1;
            toMultiply[0] = 2;
            toMultiply[1] = 0;
        }
    } else if (!axesNames.first.tolower().compare("y")) {
        map[0] = 1;
        if (!axesNames.second.tolower().compare("x")) {
            map[1] = 0;
            map[2] = 2;
            toMultiply[0] = 0;
            toMultiply[1] = 1;
        } else {
            map[1] = 2;
            map[2] = 0;
            toMultiply[0] = 1;
            toMultiply[1] = 2;
        }
    } else if (!axesNames.first.tolower().compare("z")) {
        map[0] = 2;
        if (!axesNames.second.tolower().compare("x")) {
            map[1] = 0;
            map[2] = 1;
            toMultiply[0] = 2;
            toMultiply[1] = 0;
        } else {
            map[1] = 1;
            map[2] = 0;
            toMultiply[0] = 1;
            toMultiply[1] = 2;
        }
    }

    // Get the system of axis XYZ
    std::vector<biorbd::utils::Vector3d> axes(3);
    axes[map[0]] = axis1markers.second - axis1markers.first;
    axes[map[1]] = axis2markers.second - axis2markers.first;
    axes[map[2]] = axes[toMultiply[0]].cross(axes[toMultiply[1]]);

    // Recalculate one axis
    if (!axisToRecalculate.tolower().compare("x")) {
        axes[0] = axes[1].cross(axes[2]);
    } else if (!axisToRecalculate.tolower().compare("y")) {
        axes[1] = axes[2].cross(axes[0]);
    } else if (!axisToRecalculate.tolower().compare("z")) {
        axes[2] = axes[0].cross(axes[1]);
    }

    // Organize them in a non-normalized matrix
    biorbd::utils::Matrix r_out(3, 3);
    for (unsigned int i=0; i<3; ++i) {
        r_out.block(0, i, 3, 1) = axes[i];
    }
    return r_out;
}

biorbd::utils::Rotation biorbd::utils::Rotation::fromMarkers(
    const std::pair<biorbd::rigidbody::NodeSegment, biorbd::rigidbody::NodeSegment>
    &axis1markers,
    const std::pair<biorbd::rigidbody::NodeSegment, biorbd::rigidbody::NodeSegment>
    &axis2markers,
    const std::pair<biorbd::utils::String, biorbd::utils::String>& axesNames,
    const biorbd::utils::String &axisToRecalculate)
{
    biorbd::utils::Matrix r_out(
        fromMarkersNonNormalized(axis1markers, axis2markers, axesNames,
                                 axisToRecalculate));

    // Organize them in a normalized matrix
    for (unsigned int i=0; i<3; ++i) {
        // Normalize axes
        r_out.block<3, 1>(0, i).normalize();
    }

    return r_out;
}

biorbd::utils::Vector biorbd::utils::Rotation::toEulerAngles(
    const biorbd::utils::Rotation &r,
    const biorbd::utils::String &seq)
{
    biorbd::utils::Vector v;
    if (!seq.compare("zyzz")) {
        v = biorbd::utils::Vector(3);
    } else {
        v = biorbd::utils::Vector(static_cast<unsigned int>(seq.length()));
    }

    if (!seq.compare("x")) {
        v[0] = std::asin(r(2, 1));           // x
    } else if (!seq.compare("y")) {
        v[0] = std::asin(r(0, 2));           // y
    } else if (!seq.compare("z")) {
        v[0] = std::asin(r(1, 0));           // z
    } else if (!seq.compare("xy")) {
        v[0] = std::asin(r(2,1));            // x
        v[1] = std::asin(r(0,2));            // y
    } else if (!seq.compare("xz")) {
        v[0] = -std::asin(r(1,2));           // x
        v[1] = -std::asin(r(0,1));           // z
    } else if (!seq.compare("yx")) {
        v[0] = -std::asin(r(2,0));           // y
        v[1] = -std::asin(r(1,2));           // x
    } else if (!seq.compare("yz")) {
        v[0] = std::asin(r(0,2));            // y
        v[1] = std::asin(r(1,0));            // z
    } else if (!seq.compare("zx")) {
        v[0] = std::asin(r(1,0));            // z
        v[1] = std::asin(r(2,1));            // x
    } else if (!seq.compare("zy")) {
        v[0] = -std::asin(r(0,1));           // z
        v[1] = -std::asin(r(2,0));           // y
    } else if (!seq.compare("xyz")) {
        v[0] = std::atan2(-r(1,2), r(2,2));  // x
        v[1] = std::asin(r(0,2));            // y
        v[2] = std::atan2(-r(0,1), r(0,0));  // z
    } else if (!seq.compare("xzy")) {
        v[0] = std::atan2(r(2,1), r(1,1));   // x
        v[1] = std::asin(-r(0,1));           // z
        v[2] = std::atan2(r(0,2), r(0,0));   // y
    } else if (!seq.compare("yxz")) {
        v[0] = std::atan2(r(0,2), r(2,2));   // y
        v[1] = std::asin(-r(1,2));           // x
        v[2] = std::atan2(r(1,0), r(1,1));   // z
    } else if (!seq.compare("yzx")) {
        v[0] = std::atan2(-r(2,0), r(0,0));  // y
        v[1] = std::asin(r(1,0));            // z
        v[2] = std::atan2(-r(1,2), r(1,1));  // x
    } else if (!seq.compare("zxy")) {
        v[0] = std::atan2(-r(0,1), r(1,1));  // z
        v[1] = std::asin(r(2,1));            // x
        v[2] = std::atan2(-r(2,0), r(2,2));  // y
    } else if (!seq.compare("zyx")) {
        v[0] = std::atan2(r(1,0), r(0,0));   // z
        v[1] = std::asin(-r(2,0));           // y
        v[2] = std::atan2(r(2,1), r(2,2));   // x
    } else if (!seq.compare("zxz")) {
        v[0] = std::atan2(r(0,2), -r(1,2));  // z
        v[1] = std::acos(r(2,2));            // x
        v[2] = std::atan2(r(2,0), r(2,1));   // z
    } else if (!seq.compare("zyz")) {
        v[0] = std::atan2(r(1,2), r(0,2));   // z
        v[1] = std::acos(r(2,2));            // y
        v[2] = std::atan2(r(2,1), -r(2,0));  // z
    } else if (!seq.compare("zyzz")) {
        v[0] = std::atan2(r(1,2), r(0,2));   // z
        v[1] = std::acos(r(2,2));            // y
        v[2] = std::atan2(r(2,1), -r(2,0)) + v[0];   // z+z
    } else {
        biorbd::utils::Error::raise("Angle sequence is not recognized");
    }

    return v;
}

#ifndef BIORBD_USE_CASADI_MATH
biorbd::utils::Rotation biorbd::utils::Rotation::mean(
    const std::vector<biorbd::utils::Rotation> & mToMean)
{
    RigidBodyDynamics::Math::Matrix3d m_tp;
    m_tp.setZero();

    // Initial guess being the actual arithmetic mean
    for (unsigned int i = 0; i<mToMean.size(); ++i) {
        m_tp += mToMean[i];
    }
    m_tp = m_tp/mToMean.size();

    // SVD decomposition
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        m_tp, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Normalize the matrix
    biorbd::utils::Rotation m_out(svd.matrixU() * svd.matrixV().transpose());

    return m_out;
}
#endif

void biorbd::utils::Rotation::checkUnitary()
{
#ifndef BIORBD_USE_CASADI_MATH
#ifndef SKIP_ASSERT
    double sqrtNorm = static_cast<double>(this->squaredNorm());
    biorbd::utils::Error::check(fabs(sqrtNorm - 3.) < 1e-4,
                                biorbd::utils::String("The Rotation matrix square norm is equal to ")
                                + sqrtNorm + ", but should be equal to 3");
#endif
#endif
}

std::ostream &operator<<(std::ostream &os, const biorbd::utils::Rotation &a)
{
    os << a.block(0,0,3,3);
    return os;
}
