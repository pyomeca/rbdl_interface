#define BIORBD_API_EXPORTS
#include "Utils/Quaternion.h"

#include "Utils/Vector3d.h"
#include "Utils/Vector.h"
#include "Utils/RotoTrans.h"
#include "Utils/Error.h"
#include "Utils/Rotation.h"

biorbd::utils::Quaternion::Quaternion (
    double kStabilizer) :
    RigidBodyDynamics::Math::Vector4d (1, 0, 0, 0),
    m_Kstab(kStabilizer)
{

}

biorbd::utils::Quaternion::Quaternion(
    const biorbd::utils::Quaternion &other) :
    RigidBodyDynamics::Math::Vector4d (other),
    m_Kstab(other.m_Kstab)
{

}

biorbd::utils::Quaternion::Quaternion(
    const RigidBodyDynamics::Math::Vector4d &vec4,
    double kStabilizer) :
    RigidBodyDynamics::Math::Vector4d (vec4),
    m_Kstab(kStabilizer)
{

}

biorbd::utils::Quaternion::Quaternion (
    const biorbd::utils::Scalar& w,
    const biorbd::utils::Scalar& x,
    const biorbd::utils::Scalar& y,
    const biorbd::utils::Scalar& z,
    double kStabilizer) :
    RigidBodyDynamics::Math::Vector4d(w, x, y, z),
    m_Kstab(kStabilizer)
{

}

biorbd::utils::Quaternion::Quaternion (
    const biorbd::utils::Scalar& w,
    const biorbd::utils::Vector3d &vec3,
    double kStabilizer) :
    RigidBodyDynamics::Math::Vector4d(w, vec3[0], vec3[1], vec3[2]),
    m_Kstab(kStabilizer)
{

}

biorbd::utils::Scalar biorbd::utils::Quaternion::w() const
{
    return (*this)(0);
}
biorbd::utils::Scalar biorbd::utils::Quaternion::x() const
{
    return (*this)(1);
}
biorbd::utils::Scalar biorbd::utils::Quaternion::y() const
{
    return (*this)(2);
}
biorbd::utils::Scalar biorbd::utils::Quaternion::z() const
{
    return (*this)(3);
}

void biorbd::utils::Quaternion::setKStab(double newKStab)
{
    m_Kstab = newKStab;
}

double biorbd::utils::Quaternion::kStab() const
{
    return m_Kstab;
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::operator*(
    const biorbd::utils::Quaternion& q) const
{
    return biorbd::utils::Quaternion (
               (*this)[0] * q[0] - (*this)[1] * q[1] - (*this)[2] * q[2] - (*this)[3] * q[3],
               (*this)[0] * q[1] + (*this)[1] * q[0] + (*this)[2] * q[3] - (*this)[3] * q[2],
               (*this)[0] * q[2] + (*this)[2] * q[0] + (*this)[3] * q[1] - (*this)[1] * q[3],
               (*this)[0] * q[3] + (*this)[3] * q[0] + (*this)[1] * q[2] - (*this)[2] * q[1],
               (this->m_Kstab + q.m_Kstab) / 2);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::operator*(
    const biorbd::utils::Scalar& scalar) const
{
    return biorbd::utils::Quaternion (
               this->RigidBodyDynamics::Math::Vector4d::operator*(scalar), this->m_Kstab);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::operator*(
    float scalar) const
{
    return biorbd::utils::Quaternion (
               this->RigidBodyDynamics::Math::Vector4d::operator*(
                   static_cast<double>(scalar)), this->m_Kstab);
}

#ifdef BIORBD_USE_CASADI_MATH
biorbd::utils::Quaternion biorbd::utils::Quaternion::operator*(
    double scalar) const
{
    return biorbd::utils::Quaternion (
               this->RigidBodyDynamics::Math::Vector4d::operator*(scalar),
               this->m_Kstab);
}
#endif

biorbd::utils::Quaternion biorbd::utils::Quaternion::operator+(
    const biorbd::utils::Quaternion& other) const
{
    return biorbd::utils::Quaternion(
               this->RigidBodyDynamics::Math::Vector4d::operator+(other),
               (this->m_Kstab + other.m_Kstab) / 2);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::operator-(
    const biorbd::utils::Quaternion& other) const
{
    return biorbd::utils::Quaternion(
               this->RigidBodyDynamics::Math::Vector4d::operator-(other),
               (this->m_Kstab + other.m_Kstab) / 2);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::fromGLRotate(
    const biorbd::utils::Scalar& angle,
    const biorbd::utils::Scalar& x,
    const biorbd::utils::Scalar& y,
    const biorbd::utils::Scalar& z,
    double kStab)
{
    biorbd::utils::Scalar angle_copy(angle);
    biorbd::utils::Scalar st = std::sin (angle_copy * M_PI / 360.);
    return biorbd::utils::Quaternion (
               std::cos (angle_copy * M_PI / 360.), st * x, st * y, st * z, kStab);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::fromAxisAngle(
    const biorbd::utils::Scalar& angle,
    const biorbd::utils::Vector3d &axis,
    double kStab)
{
    biorbd::utils::Scalar angle_copy(angle);
    biorbd::utils::Scalar d = axis.norm();
    biorbd::utils::Scalar s2 = std::sin (angle_copy * 0.5) / d;
    return biorbd::utils::Quaternion (
               std::cos(angle_copy * 0.5),
               axis[0] * s2, axis[1] * s2, axis[2] * s2, kStab
           );
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::fromMatrix(
    const biorbd::utils::RotoTrans &rt,
    double kStab)
{
    return fromMatrix(rt.rot(), kStab);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::fromMatrix(
    const biorbd::utils::Rotation &mat,
    double kStab)
{
    biorbd::utils::Scalar w = std::sqrt (1. + mat(0,0) + mat(1,1) + mat(2,2)) * 0.5;
    return Quaternion (
               w,
               (mat(2,1) - mat(1,2)) / (w * 4.),
               (mat(0,2) - mat(2,0)) / (w * 4.),
               (mat(1,0) - mat(0,1)) / (w * 4.),
               kStab);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::fromZYXAngles(
    const biorbd::utils::Vector3d &zyx_angles,
    double kStab)
{
    return fromAxisAngle (zyx_angles[2], biorbd::utils::Vector3d (0., 0., 1.),
                          kStab)
           * fromAxisAngle (zyx_angles[1], biorbd::utils::Vector3d (0., 1., 0.), kStab)
           * fromAxisAngle (zyx_angles[0], biorbd::utils::Vector3d (1., 0., 0.), kStab);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::fromYXZAngles(
    const biorbd::utils::Vector3d &yxz_angles,
    double kStab)
{
    return fromAxisAngle (yxz_angles[1], biorbd::utils::Vector3d (0., 1., 0.),
                          kStab)
           * fromAxisAngle (yxz_angles[0], biorbd::utils::Vector3d (1., 0., 0.), kStab)
           * fromAxisAngle (yxz_angles[2], biorbd::utils::Vector3d (0., 0., 1.), kStab);
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::fromXYZAngles(
    const biorbd::utils::Vector3d &xyz_angles,
    double kStab)
{
    return fromAxisAngle (xyz_angles[0], biorbd::utils::Vector3d (1., 0., 0.),
                          kStab)
           * fromAxisAngle (xyz_angles[1], biorbd::utils::Vector3d (0., 1., 0.), kStab)
           * fromAxisAngle (xyz_angles[2], biorbd::utils::Vector3d (0., 0., 1.), kStab);
}

biorbd::utils::Rotation biorbd::utils::Quaternion::toMatrix(
    bool skipAsserts) const
{
#ifndef BIORBD_USE_CASADI_MATH
    if (!skipAsserts) {
        biorbd::utils::Error::check(fabs(this->squaredNorm() - 1.) < 1e-10,
                                    "The Quaternion norm is not equal to one");
    }
#endif


    biorbd::utils::Scalar w = (*this)[0];
    biorbd::utils::Scalar x = (*this)[1];
    biorbd::utils::Scalar y = (*this)[2];
    biorbd::utils::Scalar z = (*this)[3];
    biorbd::utils::Rotation out = biorbd::utils::Rotation(
                                      1 - 2*y*y - 2*z*z,  2*x*y - 2*w*z,      2*x*z + 2*w*y,
                                      2*x*y + 2*w*z,      1 - 2*x*x - 2*z*z,  2*y*z - 2*w*x,
                                      2*x*z - 2*w*y,      2*y*z + 2*w*x,      1 - 2*x*x - 2*y*y);
    return out;
}

#ifndef BIORBD_USE_CASADI_MATH
biorbd::utils::Quaternion biorbd::utils::Quaternion::slerp(
    double alpha,
    const biorbd::utils::Quaternion &quat) const
{
    // check whether one of the two has 0 length
    biorbd::utils::Scalar s = std::sqrt (squaredNorm() * quat.squaredNorm());

    // division by 0.f is unhealthy!
#ifndef BIORBD_USE_CASADI_MATH
    assert (s != 0.);
#endif

    biorbd::utils::Scalar angle = acos (dot(quat) / s);
#ifdef BIORBD_USE_CASADI_MATH
    if (true) {
#else
    if (angle == 0. || std::isnan(angle)) {
#endif
        return *this;
    }

    biorbd::utils::Scalar d = 1. / std::sin (angle);
    biorbd::utils::Scalar p0 = std::sin ((1. - alpha) * angle);
    biorbd::utils::Scalar p1 = std::sin (alpha * angle);

#ifdef BIORBD_USE_CASADI_MATH
    return Quaternion(casadi::MX::if_else(
                          casadi::MX::lt(dot (quat), 0.),
                          RigidBodyDynamics::Math::Vector4d( ((*this) * p0 - quat * p1) * d),
                          RigidBodyDynamics::Math::Vector4d( ((*this) * p0 + quat * p1) * d)),
                      (this->m_Kstab + quat.m_Kstab) / 2);

#else
    if (dot (quat) < 0.) {
        return Quaternion( ((*this) * p0 - quat * p1) * d, this->m_Kstab);
    }
    return Quaternion( ((*this) * p0 + quat * p1) * d,
                       (this->m_Kstab + quat.m_Kstab) / 2);
#endif
}
#endif

biorbd::utils::Quaternion biorbd::utils::Quaternion::conjugate() const
{
    return biorbd::utils::Quaternion (
               (*this)[0],
               -(*this)[1],-(*this)[2],-(*this)[3],
               this->kStab()
           );
}

biorbd::utils::Quaternion biorbd::utils::Quaternion::timeStep(
    const biorbd::utils::Vector3d &omega,
    double dt)
{
    biorbd::utils::Scalar omega_norm = omega.norm();
    return fromAxisAngle (
               dt * omega_norm, omega / omega_norm, this->m_Kstab) * (*this);
}

biorbd::utils::Vector3d biorbd::utils::Quaternion::rotate(
    const biorbd::utils::Vector3d &vec) const
{
    biorbd::utils::Quaternion vec_quat (0., vec);

    biorbd::utils::Quaternion res_quat(vec_quat * (*this));
    res_quat = conjugate() * res_quat;

    return biorbd::utils::Vector3d(res_quat[1], res_quat[2], res_quat[3]);
}

#include <iostream>
biorbd::utils::Quaternion biorbd::utils::Quaternion::omegaToQDot(
    const biorbd::utils::Vector3d &omega) const
{
    RigidBodyDynamics::Math::MatrixNd m(4, 3);
    m(0, 0) = -(*this)[1];
    m(0, 1) = -(*this)[2];
    m(0, 2) = -(*this)[3];
    m(1, 0) =  (*this)[0];
    m(1, 1) = -(*this)[3];
    m(1, 2) =  (*this)[2];
    m(2, 0) =  (*this)[3];
    m(2, 1) =  (*this)[0];
    m(2, 2) = -(*this)[1];
    m(3, 0) = -(*this)[2];
    m(3, 1) =  (*this)[1];
    m(3, 2) =  (*this)[0];

    return biorbd::utils::Quaternion(0.5 * m * omega, this->m_Kstab);
}

biorbd::utils::Vector3d  biorbd::utils::Quaternion::eulerDotToOmega(
    const biorbd::utils::Vector3d &eulerDot,
    const biorbd::utils::Vector3d &euler,
    const biorbd::utils::String& seq)
{

    biorbd::utils::Vector3d w;
    biorbd::utils::Scalar dph, dth, dps, ph, th, ps;
    dph = eulerDot[0];
    dth = eulerDot[1];
    dps = eulerDot[2];
    ph = euler[0];
    th = euler[1];
    ps = euler[2];
    if (!seq.compare("xyz")) {          // xyz
        w[0] = dph*std::cos(th)*std::cos(ps) + dth*std::sin(ps);
        w[1] = dth*std::cos(ps) - dph*std::cos(th)*std::sin(ps);
        w[2] = dph*std::sin(th) + dps;
    } else {
        biorbd::utils::Error::raise("Angle sequence is either nor implemented or not recognized");
    }
    return w;
}

void biorbd::utils::Quaternion::derivate(
    const biorbd::utils::Vector &w)
{
    // Création du quaternion de "préproduit vectoriel"
#ifdef BIORBD_USE_CASADI_MATH
    biorbd::utils::Scalar qw = (*this)(0);
    biorbd::utils::Scalar qx = (*this)(1);
    biorbd::utils::Scalar qy = (*this)(2);
    biorbd::utils::Scalar qz = (*this)(3);
#else
    biorbd::utils::Scalar& qw = (*this)(0);
    biorbd::utils::Scalar& qx = (*this)(1);
    biorbd::utils::Scalar& qy = (*this)(2);
    biorbd::utils::Scalar& qz = (*this)(3);
#endif
    RigidBodyDynamics::Math::Matrix4d Q =
        RigidBodyDynamics::Math::Matrix4d(
            qw, -qx, -qy, -qz,
            qx,  qw, -qz,  qy,
            qy,  qz,  qw, -qx,
            qz, -qy,  qx,  qw);

    // Ajout du paramètre de stabilisation
    RigidBodyDynamics::Math::Vector4d w_tp (m_Kstab*w.norm()*(1-this->norm()), w(0),
                                            w(1), w(2));
    RigidBodyDynamics::Math::Vector4d newQuat(0.5 * Q * w_tp);

    // Assigning is slightly faster than create a new Quaternion
    qw = newQuat[0];
    qx = newQuat[1];
    qy = newQuat[2];
    qz = newQuat[3];
#ifdef BIORBD_USE_CASADI_MATH
    *this = biorbd::utils::Quaternion(qw, qx, qy, qz, m_Kstab);
#endif
}

void biorbd::utils::Quaternion::normalize()
{
    *this = *this / this->norm();
}
