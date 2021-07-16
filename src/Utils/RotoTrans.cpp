#define BIORBD_API_EXPORTS
#include "Utils/RotoTrans.h"

#include "Utils/Error.h"
#include "Utils/Vector3d.h"
#include "Utils/String.h"
#include "Utils/Vector.h"
#include "Utils/Rotation.h"

#include "RigidBody/NodeSegment.h"

biorbd::utils::RotoTrans::RotoTrans(
    const RigidBodyDynamics::Math::Matrix4d& matrix) :
    RigidBodyDynamics::Math::Matrix4d(matrix)
{
    checkUnitary();
}

biorbd::utils::RotoTrans::RotoTrans(
    const biorbd::utils::Scalar& v00, const biorbd::utils::Scalar& v01,
    const biorbd::utils::Scalar& v02,
    const biorbd::utils::Scalar& v03,
    const biorbd::utils::Scalar& v10, const biorbd::utils::Scalar& v11,
    const biorbd::utils::Scalar& v12,
    const biorbd::utils::Scalar& v13,
    const biorbd::utils::Scalar& v20, const biorbd::utils::Scalar& v21,
    const biorbd::utils::Scalar& v22,
    const biorbd::utils::Scalar& v23,
    const biorbd::utils::Scalar& v30, const biorbd::utils::Scalar& v31,
    const biorbd::utils::Scalar& v32,
    const biorbd::utils::Scalar& v33) :
    RigidBodyDynamics::Math::Matrix4d (v00, v01, v02, v03,
                                       v10, v11, v12, v13,
                                       v20, v21, v22, v23,
                                       v30, v31, v32, v33)
{
    checkUnitary();
}

biorbd::utils::RotoTrans::RotoTrans(
    const biorbd::utils::Rotation& rot) :
    RigidBodyDynamics::Math::Matrix4d(combineRotAndTrans(rot,
                                      biorbd::utils::Vector3d::Zero()))
{

}

biorbd::utils::RotoTrans::RotoTrans(
    const biorbd::utils::Rotation& rot,
    const biorbd::utils::Vector3d& trans) :
    RigidBodyDynamics::Math::Matrix4d(combineRotAndTrans(rot,trans))
{

}

biorbd::utils::RotoTrans::RotoTrans(
    const biorbd::utils::Vector& rotation,
    const biorbd::utils::Vector3d& translation,
    const biorbd::utils::String& rotationSequence) :
    RigidBodyDynamics::Math::Matrix4d(fromEulerAngles(rotation, translation,
                                      rotationSequence))
{

}

biorbd::utils::RotoTrans::RotoTrans(
    const RigidBodyDynamics::Math::SpatialTransform& st) :
    RigidBodyDynamics::Math::Matrix4d(fromSpatialTransform(st))
{

}

biorbd::utils::RotoTrans biorbd::utils::RotoTrans::fromMarkers(
    const biorbd::rigidbody::NodeSegment& origin,
    const std::pair<biorbd::rigidbody::NodeSegment, biorbd::rigidbody::NodeSegment>
    &axis1markers,
    const std::pair<biorbd::rigidbody::NodeSegment, biorbd::rigidbody::NodeSegment>
    &axis2markers,
    const std::pair<biorbd::utils::String, biorbd::utils::String>& axesNames,
    const biorbd::utils::String &axisToRecalculate)
{
    RotoTrans rt_out;
    rt_out.block(0, 0, 3, 3) = Rotation::fromMarkers(axis1markers, axis2markers,
                               axesNames, axisToRecalculate);
    rt_out.block(0, 3, 3, 1) = origin;
    return rt_out;
}

biorbd::utils::Vector3d biorbd::utils::RotoTrans::axe(unsigned int idx) const
{
    biorbd::utils::Error::check(
        idx<=2, "Axis must be between 0 and 2 included");
    return rot().block(0,idx,3,1);
}

biorbd::utils::RotoTrans biorbd::utils::RotoTrans::transpose() const
{
    biorbd::utils::RotoTrans tp;
    tp.block(0, 0, 3, 3) = block(0, 0, 3, 3).transpose();
    tp.block(0, 3, 3, 1) = -tp.block(0, 0, 3, 3) * block(0, 3, 3, 1);
    tp.block(3, 0, 1, 4) = RigidBodyDynamics::Math::Vector4d(0, 0, 0,
                           1).transpose();
    return tp;
}

biorbd::utils::Vector3d biorbd::utils::RotoTrans::trans() const
{
    return this->block<3, 1>(0,3);
}

biorbd::utils::Rotation biorbd::utils::RotoTrans::rot() const
{
    return this->block<3, 3>(0,0);
}

biorbd::utils::RotoTrans biorbd::utils::RotoTrans::combineRotAndTrans(
    const biorbd::utils::Rotation& rot,
    const biorbd::utils::Vector3d& trans)
{
    biorbd::utils::RotoTrans out;
    out.block(0,0,3,3) = rot;
    out.block(0,3,3,1) = trans;
    out.block(3, 0, 1, 4) = RigidBodyDynamics::Math::Vector4d(0, 0, 0,
                            1).transpose();
    return out;
}

biorbd::utils::RotoTrans biorbd::utils::RotoTrans::fromSpatialTransform(
    const RigidBodyDynamics::Math::SpatialTransform& st)
{
    return combineRotAndTrans(st.E,st.r);
}

biorbd::utils::RotoTrans biorbd::utils::RotoTrans::fromEulerAngles(
    const biorbd::utils::Vector& rot,
    const biorbd::utils::Vector3d& trans,
    const biorbd::utils::String& seq)
{

    biorbd::utils::Rotation rot_mat(biorbd::utils::Rotation::fromEulerAngles(rot,
                                    seq));

    biorbd::utils::RotoTrans out;
    out.block(0, 0, 3, 3) = rot_mat;
    out.block(0, 3, 3, 1) = trans;
    return out;
}

biorbd::utils::Vector biorbd::utils::RotoTrans::toEulerAngles(
    const biorbd::utils::RotoTrans& rt,
    const biorbd::utils::String &seq)
{
    return biorbd::utils::Rotation::toEulerAngles(rt.block<3, 3>(0, 0), seq);
}

#ifndef BIORBD_USE_CASADI_MATH
biorbd::utils::RotoTrans biorbd::utils::RotoTrans::mean(
    const std::vector<RotoTrans> & mToMean)
{
    // The translation part is just the actual mean
    RigidBodyDynamics::Math::Vector3d v_tp;
    v_tp.setZero();
    for (unsigned int i = 0; i<mToMean.size(); ++i) {
        v_tp += mToMean[i].trans();
    }
    v_tp = v_tp/mToMean.size();

    // The rotation part should call the proper way implemented in Rotation
    std::vector<biorbd::utils::Rotation> rotations;
    for (unsigned int i=0; i<mToMean.size(); ++i) {
        rotations.push_back(mToMean[i].block<3, 3>(0, 0));
    }
    biorbd::utils::RotoTrans m_out(
        biorbd::utils::Rotation::mean(rotations), v_tp);
    return m_out;
}
#endif

RigidBodyDynamics::Math::Vector4d biorbd::utils::RotoTrans::expand3dTo4d(
    const biorbd::utils::Vector3d &v1)
{
    RigidBodyDynamics::Math::Vector4d v2;
    v2.block(0,0,3,1) = v1;
    v2(3) = 1;
    return v2;
}

void biorbd::utils::RotoTrans::checkUnitary()
{
#ifndef BIORBD_USE_CASADI_MATH
#ifndef SKIP_ASSERT
    this->rot(); // Automatically cast the test for the rotation part
    biorbd::utils::Error::check(this->block(3, 0, 1, 4).sum() == 1.,
                                "Last row of the RotoTrans should be (0,0,0,1");
    biorbd::utils::Error::check((*this)(3, 3) == 1.,
                                "Last row of the RotoTrans should be (0,0,0,1");
#endif
#endif
}

std::ostream &operator<<(std::ostream &os, const biorbd::utils::RotoTrans &a)
{
    os << a.block(0,0,4,4);
    return os;
}
