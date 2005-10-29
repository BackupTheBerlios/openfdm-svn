/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Quaternion_H
#define OpenFDM_Quaternion_H

#include <iosfwd>

#include "Assert.h"
#include "Types.h"
#include "Limits.h"
#include "Units.h"
#include "Vector.h"
#include "Matrix.h"

namespace OpenFDM {

namespace LinAlg {

// Forward decls.
template<typename T>
class Quaternion;

template<typename T>
OpenFDM_FORCE_INLINE
Quaternion<T>
operator*(const Quaternion<T>& q1, const Quaternion<T>& q2);

template<typename T>
class Quaternion
  : public Vector4<T> {
public:
  typedef typename Vector4<T>::value_type value_type;
  typedef Vector3<value_type> Vector3;
  typedef Matrix33<value_type> Matrix33;
  
  OpenFDM_FORCE_INLINE
  Quaternion(void)
  {}
  OpenFDM_FORCE_INLINE
  Quaternion(const Vector4<T>& v)
    : Vector4<T>(v)
  {}
  OpenFDM_FORCE_INLINE
  Quaternion(value_type q1, value_type q2, value_type q3, value_type q4)
    : Vector4<T>(q1, q2, q3, q4)
  {}
  OpenFDM_FORCE_INLINE
  ~Quaternion(void)
  {}

  OpenFDM_FORCE_INLINE
  bool isIdentity(void) const
  {
    return fabs(fabs((*this)(1))-1) < Limits<T>::epsilon() &&
      fabs((*this)(2)) < Limits<T>::epsilon() &&
      fabs((*this)(3)) < Limits<T>::epsilon() &&
      fabs((*this)(4)) < Limits<T>::epsilon();
  }

  void setAngleAxis(value_type angle, const Vector3& axis)
  { (*this) = Quaternion::fromAngleAxis(angle, axis); }
  void setAngleAxisDeg(value_type deg, const Vector3& axis)
  { setAngleAxis(deg*deg2rad, axis); }

  Vector3 getAxis(void) const
  {
    value_type nrm = norm(*this);
    // More or less emergency exit. Should not happen ...
    if (nrm < Limits<value_type>::epsilon())
      return Vector3::zeros();

    Vector4<T> nq = (*this)/nrm;
    value_type cosAngle2 = nq(1);
    Vector3 axis = Vector3(nq(2), nq(3), nq(4));
    value_type sinAngle2 = norm(axis);
    axis = normalize(axis);
    if (-0.7 < sinAngle2 && sinAngle2 < 0.7)
      return 2.0*asin(sinAngle2)*axis;
    else
      return 2.0*acos(cosAngle2)*axis;
  }
  void setAxis(const Vector3& axis)
  { (*this) = Quaternion::fromAxis(axis); }

  void setRotateTo(const Vector3& from, const Vector3& to)
  { (*this) = Quaternion::fromRotateTo(from, to); }
  void setRotateTo(size_type i, const Vector3& v)
  { (*this) = Quaternion::fromRotateTo(i, v); }
  void setRotateTo(size_type i1, const Vector3& v1,
                   size_type i2, const Vector3& v2)
  { (*this) = Quaternion::fromRotateTo(i1, v1, i2, v2); }

  Vector3 getEuler(void) const
  {
    Vector3 angles;

    value_type q1 = (*this)(1);
    value_type q2 = (*this)(2);
    value_type q3 = (*this)(3);
    value_type q4 = (*this)(4);
    value_type sqrQ1 = q1*q1;
    value_type sqrQ2 = q2*q2;
    value_type sqrQ3 = q3*q3;
    value_type sqrQ4 = q4*q4;

    value_type tmp = sqrQ1 - sqrQ2 - sqrQ3 + sqrQ4;
    if (fabs(tmp) < Limits<value_type>::min())
      angles(1) = pi05;
    else
      angles(1) = atan2(2*(q3*q4 + q1*q2), tmp);
    
    tmp = 2*(q2*q4 - q1*q3);
    if (tmp < -1.0)
      angles(2) = pi05;
    else if (1.0 < tmp)
      angles(2) = -pi05;
    else
      angles(2) = -asin(tmp);
    
    tmp = sqrQ1 + sqrQ2 - sqrQ3 - sqrQ4;
    if (fabs(tmp) < Limits<value_type>::min())
      angles(3) = pi05;
    else {
      value_type psi = atan2(2*(q2*q3 + q1*q4), tmp);
      if (psi < 0.0)
        psi += pi2;
      angles(3) = psi;
    }

    return angles;
  }

  /** Arithmetic operator "*=".
      Inplace quaternion multiplication.
      @param q a quaternion to be multiplied.
      @return reference to *this. */
  OpenFDM_FORCE_INLINE
  Quaternion& operator*=(const Quaternion& q)
  { return (*this) = (*this)*q; }

  /** Arithmetic operator "/=".
      Inplace quaternion multiplication.
      @param q a quaternion to be multiplied.
      @return reference to *this. */
  OpenFDM_FORCE_INLINE
  Quaternion& operator/=(const Quaternion& q)
  { return (*this) = (*this)*inverse(q); }

  OpenFDM_FORCE_INLINE
  Matrix33 getTransform(void) const
  {
    value_type q1 = (*this)(1);
    value_type q2 = (*this)(2);
    value_type q3 = (*this)(3);
    value_type q4 = (*this)(4);

    value_type r = 1/dot(*this, *this);
    value_type rq1 = r*q1;
    value_type rq2 = r*q2;
    value_type rq3 = r*q3;
    value_type rq4 = r*q4;

    // Now compute the transformation matrix.
    value_type q1q1 = rq1*q1;
    value_type q2q2 = rq2*q2;
    value_type q3q3 = rq3*q3;
    value_type q4q4 = rq4*q4;
    value_type q1q2 = rq1*q2;
    value_type q1q3 = rq1*q3;
    value_type q1q4 = rq1*q4;
    value_type q2q3 = rq2*q3;
    value_type q2q4 = rq2*q4;
    value_type q3q4 = rq3*q4;
    
    return Matrix33(q1q1+q2q2-q3q3-q4q4, 2*(q2q3+q1q4), 2*(q2q4-q1q3),
                    2*(q2q3-q1q4), q1q1-q2q2+q3q3-q4q4, 2*(q3q4+q1q2),
                    2*(q2q4+q1q3), 2*(q3q4-q1q2), q1q1-q2q2-q3q3+q4q4);
  }
  OpenFDM_FORCE_INLINE
  Matrix33 getBackTransform(void) const
  { return trans(getTransform()); }

  Vector3 transform(const Vector3& v) const
  {
    value_type r = 1/dot(*this, *this);
    Vector3 qimag = imag(*this);
    value_type qr = real(*this);
    return (2*r*qr*qr - 1)*v
      + (2*r*dot(qimag, v))*qimag
      - (2*r*qr)*cross(qimag, v);
  }
  Vector3 backTransform(const Vector3& v) const
  {
    value_type r = 1/dot(*this, *this);
    Vector3 qimag = imag(*this);
    value_type qr = real(*this);
    return (2*r*qr*qr - 1)*v
      + (2*r*dot(qimag, v))*qimag
      + (2*r*qr)*cross(qimag, v);
  }


  static Quaternion fromRealImag(value_type r, const Vector3& i)
  { return Quaternion(r, i(1), i(2), i(3)); }
  static Quaternion fromImag(const Vector3& i)
  { return Quaternion(0, i(1), i(2), i(3)); }
  static Quaternion fromReal(value_type r)
  { return Quaternion(r, 0, 0, 0); }

  static Quaternion fromEulerSeq(unsigned i, value_type angle)
  { return Quaternion::fromAngleAxis(angle, Vector3::unit(i)); }
  static Quaternion fromEulerSeq(unsigned i1, value_type angle1,
                                 unsigned i2, value_type angle2)
  {
    Quaternion q1 = Quaternion::fromEulerSeq(i1, angle1);
    Quaternion q2 = Quaternion::fromEulerSeq(i2, angle2);
    return q1*q2;
  }
  static Quaternion fromEulerSeq(unsigned i1, value_type angle1,
                                 unsigned i2, value_type angle2,
                                 unsigned i3, value_type angle3)
  {
    Quaternion q1 = Quaternion::fromEulerSeq(i1, angle1);
    Quaternion q2 = Quaternion::fromEulerSeq(i2, angle2);
    Quaternion q3 = Quaternion::fromEulerSeq(i3, angle3);
    return q1*q2*q3;
  }

  static Quaternion fromEuler(value_type z, value_type y, value_type x)
  {
    value_type zd2 = 0.5*z;
    value_type yd2 = 0.5*y;
    value_type xd2 = 0.5*x;
    
    value_type Szd2 = sin(zd2);
    value_type Syd2 = sin(yd2);
    value_type Sxd2 = sin(xd2);
    
    value_type Czd2 = cos(zd2);
    value_type Cyd2 = cos(yd2);
    value_type Cxd2 = cos(xd2);
    
    value_type Cxd2Czd2 = Cxd2*Czd2;
    value_type Cxd2Szd2 = Cxd2*Szd2;
    value_type Sxd2Szd2 = Sxd2*Szd2;
    value_type Sxd2Czd2 = Sxd2*Czd2;
    
    return Quaternion( Cxd2Czd2*Cyd2 + Sxd2Szd2*Syd2,
                       Sxd2Czd2*Cyd2 - Cxd2Szd2*Syd2,
                       Cxd2Czd2*Syd2 + Sxd2Szd2*Cyd2,
                       Cxd2Szd2*Cyd2 - Sxd2Czd2*Syd2);
  }

  static Quaternion fromYawPitchRoll(value_type y, value_type p, value_type r)
  { return fromEuler(y, p, r); }

  static Quaternion fromHeadAttBank(value_type h, value_type a, value_type b)
  { return fromEuler(h, a, b); }

  static Quaternion fromLonLat(value_type lon, value_type lat)
  {
    value_type zd2 = 0.5*lon;
    value_type yd2 = - pi025 - 0.5*lat;
    
    value_type Szd2 = sin(zd2);
    value_type Syd2 = sin(yd2);
    
    value_type Czd2 = cos(zd2);
    value_type Cyd2 = cos(yd2);
    
    return Quaternion( Czd2*Cyd2, -Szd2*Syd2, Czd2*Syd2, Szd2*Cyd2);
  }

  static Quaternion fromAngleAxis(value_type angle, const Vector3& axis)
  {
    value_type angle2 = 0.5*angle;
    return Quaternion::fromRealImag(cos(angle2), sin(angle2)*axis);
  }

  static Quaternion fromAngleAxisDeg(value_type deg, const Vector3& axis)
  { return Quaternion::fromAngleAxis(deg*deg2rad, axis); }

  static Quaternion fromAxis(const Vector3& axis)
  { return Quaternion::fromAngleAxis(norm(axis), normalize(axis)); }

  static Quaternion fromRotateTo(const Vector3& from, const Vector3& to)
  {
    value_type nfrom = norm(from);
    value_type nto = norm(to);
    if (nfrom < Limits<T>::min() || nto < Limits<T>::min())
      return Quaternion::unit(1);

    return Quaternion::fromRotateToNorm((1/nfrom)*from, (1/nto)*to);
  }

  static Quaternion fromRotateTo(size_type i, const Vector3& v)
  {
    value_type nv = norm(v);
    if (nv < Limits<T>::min())
      return Quaternion::unit(1);
    
    return Quaternion::fromRotateToNorm(v/nv, Vector3::unit(i));
  }

  // FIXME more finegrained error behavour.
  static Quaternion fromRotateTo(size_type i1, const Vector3& v1,
                                 size_type i2, const Vector3& v2)
  {
    value_type nrmv1 = norm(v1);
    value_type nrmv2 = norm(v2);
    if (nrmv1 < Limits<T>::min() || nrmv2 < Limits<T>::min())
      return Quaternion::unit(1);

    Vector3 nv1 = (1/nrmv1)*v1;
    Vector3 nv2 = (1/nrmv2)*v2;
    value_type dv1v2 = dot(nv1, nv2);
    if (fabs(fabs(dv1v2)-1) < Limits<value_type>::epsilon())
      return Quaternion::unit(1);

    // The first rotation can be done with the usual routine.
    Quaternion q = Quaternion::fromRotateToNorm(Vector3::unit(i1), nv1);

    // Make nv2 orthogonal to nv1.
    nv2 = nv2 - dv1v2*nv1;
    nv2 *= 1/norm(nv2);

    // For rotation of the second axis, we need to preserve the picture
    // of the first one.
    Vector3 tui2 = q.transform(Vector3::unit(i2));
    Vector3 tnv2 = q.transform(nv2);

    // FIXME there is a possibility that copysign is not correct ...
    // true??
    value_type cosang = dot(tui2, tnv2);

    value_type cos05ang = max(static_cast<value_type>(0.5+0.5*cosang),
                              static_cast<value_type>(0));
    cos05ang = sqrt(cos05ang);

    value_type sig = dot(nv1, cross(tui2, tnv2));
    value_type sin05ang = max(static_cast<value_type>(0.5-0.5*cosang),
                              static_cast<value_type>(0));
    sin05ang = copysign(sqrt(sin05ang), sig);
    q *= Quaternion::fromRealImag(cos05ang, sin05ang*nv1);

    return q;
  }


//   OpenFDM_FORCE_INLINE
//   static Quaternion zeros(void)
//   { return Quaternion(Vector4<T>::zeros()); }
  OpenFDM_FORCE_INLINE
  static Quaternion unit(unsigned i = 1)
  { return Quaternion(Vector4<T>::unit(i)); }


  // Return a quaternion which rotates the vector given by v
  // to the vector -v. Other directions are *not* preserved.
  static Quaternion fromChangeSign(const Vector3& v)
  {
    // The vector from points to the oposite direction than to.
    // Find a vector perpandicular to the vector to.
    value_type absv1 = fabs(v(1));
    value_type absv2 = fabs(v(2));
    value_type absv3 = fabs(v(3));
    
    Vector3 axis;
    if (absv2 < absv1 && absv3 < absv1) {
      value_type quot = v(2)/v(1);
      axis = (1/sqrt(1+quot*quot))*Vector3(quot, 1, 0);
    } else if (absv1 < absv2 && absv3 < absv2) {
      value_type quot = v(3)/v(2);
      axis = (1/sqrt(1+quot*quot))*Vector3(0, quot, 1);
    } else if (absv1 < absv3 && absv2 < absv3) {
      value_type quot = v(1)/v(3);
      axis = (1/sqrt(1+quot*quot))*Vector3(1, 0, quot);
    } else {
      // The all zero case.
      return Quaternion::unit(1);
    }

    return Quaternion::fromRealImag(0, axis);
  }

private:

  // Private because it assumes normalized inputs.
  static Quaternion
  fromRotateToSmaller90Deg(value_type cosang,
                           const Vector3& from, const Vector3& to)
  {
    // In this function we assume that the angle required to rotate from
    // the vector from to the vector to is <= 90 deg.
    // That is done so because of possible instabilities when we rotate more
    // then 90deg.

    // Note that the next comment does actually cover a *more* *general* case
    // than we need in this function. That shows that this formula is even
    // valid for rotations up to 180deg.

    // Because of the signs in the axis, it is sufficient to care for angles
    // in the interval [-pi,pi]. That means that 0.5*angle is in the interval
    // [-pi/2,pi/2]. But in that range the cosine is allways >= 0.
    // So we do not need to care for egative roots in the following equation:
    value_type cos05ang = sqrt(0.5+0.5*cosang);


    // Now our assumption of angles <= 90 deg comes in play.
    // For that reason, we know that cos05ang is not zero.
    // It is even more, we can see from the above formula that 
    // sqrt(0.5) < cos05ang.


    // Compute the rotation axis, that is
    // sin(angle)*normalized rotation axis
    Vector3 axis = cross(from, to);

    // We need sin(0.5*angle)*normalized rotation axis.
    // So rescale with sin(0.5*x)/sin(x).
    // To do that we use the equation:
    // sin(x) = 2*sin(0.5*x)*cos(0.5*x)
    return Quaternion::fromRealImag( cos05ang, (1/(2*cos05ang))*axis);
  }

  // Private because it assumes normalized inputs.
  static Quaternion
  fromRotateToNorm(const Vector3& from, const Vector3& to)
  {
    // To avoid instabilities with roundoff, we distinguish between rotations
    // with more then 90deg and rotations with less than 90deg.

    // Compute the cosine of the angle.
    value_type cosang = dot(from, to);

    // For the small ones do direct computation
    if (-0.5 < cosang)
      return Quaternion::fromRotateToSmaller90Deg(cosang, from, to);

    // For larger rotations. first rotate from to -from.
    // Past that we will have a smaller angle again.
    Quaternion q1 = Quaternion::fromChangeSign(from);
    Vector3 t2 = q1.transform(to);
    Vector3 f2 = -from;
    value_type cosang2 = dot(f2, t2);
    Quaternion q2 = Quaternion::fromRotateToSmaller90Deg(cosang2, f2, t2);

    return q1*q2;
  }
};

/** Arithmetic operator *.
    Multiplication of two quaternions.
    @param q1 a quaternion to be multiplied.
    @param q2 a quaternion to be multiplied.
    @return a quaternion representing q1*q2.
*/
template<typename T>
OpenFDM_FORCE_INLINE
Quaternion<T>
operator*(const Quaternion<T>& q1, const Quaternion<T>& q2)
{
  Quaternion<T> q;

  q(1) = q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4);
  q(2) = q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3);
  q(3) = q1(1)*q2(3) - q1(2)*q2(4) + q1(3)*q2(1) + q1(4)*q2(2);
  q(4) = q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2) + q1(4)*q2(1);

  return q;
}

template<typename T>
OpenFDM_FORCE_INLINE
Quaternion<T>
operator/(const Quaternion<T>& q1, const Quaternion<T>& q2)
{ return q1*inverse(q2); }

template<typename T>
OpenFDM_FORCE_INLINE
Quaternion<T>
conjugate(const Quaternion<T>& q)
{ return Quaternion<T>(q(1), -q(2), -q(3), -q(4)); }

template<typename T>
OpenFDM_FORCE_INLINE
Quaternion<T>
inverse(const Quaternion<T>& q)
{ return Quaternion<T>((1/dot(q,q))*conjugate(q)); }

template<typename T>
OpenFDM_FORCE_INLINE
typename Quaternion<T>::value_type
real(const Quaternion<T>& q)
{ return q(1); }

template<typename T>
OpenFDM_FORCE_INLINE
typename Quaternion<T>::Vector3
imag(const Quaternion<T>& q)
{ return typename Quaternion<T>::Vector3(q(2), q(3), q(4)); }

template<typename T>
OpenFDM_FORCE_INLINE
Vector4<T>
derivative(const Quaternion<T>& q, const Vector3<T>& angVel)
{
  Vector4<T> deriv;

  deriv(1) = 0.5*(-q(2)*angVel(1) - q(3)*angVel(2) - q(4)*angVel(3));
  deriv(2) = 0.5*( q(1)*angVel(1) - q(4)*angVel(2) + q(3)*angVel(3));
  deriv(3) = 0.5*( q(4)*angVel(1) + q(1)*angVel(2) - q(2)*angVel(3));
  deriv(4) = 0.5*(-q(3)*angVel(1) + q(2)*angVel(2) + q(1)*angVel(3));

  return deriv;
}

template<typename T>
bool
equal(const Quaternion<T>& q1, const Quaternion<T>& q2)
{
  T eps = Limits<T>::epsilon();
  T sqreps = eps*eps*16*16;
  Vector4<T> t1 = q1 - q2;
  Vector4<T> t2 = q1 + q2;
  return dot(t1, t1) < sqreps || dot(t2, t2) < sqreps;
}

template<typename T>
std::ostream&
operator<<(std::ostream& os, const Quaternion<T>& q)
{
  return os << trans(q) << " (Euler = " << trans(rad2deg*q.getEuler()) << ")";
}

} // namespace LinAlg

typedef LinAlg::Quaternion<real_type> Quaternion;

} // namespace OpenFDM

#endif
