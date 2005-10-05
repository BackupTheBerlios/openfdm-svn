/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Plane_H
#define OpenFDM_Plane_H

#include "Types.h"
#include "Limits.h"
#include "Vector.h"

namespace OpenFDM {

class Plane {
public:
  typedef Vector3::value_type value_type;

  Plane(void)
    : mNormal(Vector3::unit(1)), mDist(0)
  {}
  Plane(const Plane& plane)
    : mNormal(plane.mNormal), mDist(plane.mDist)
  {}
  Plane(const Vector3& normal, value_type dist)
    : mNormal(normal), mDist(dist)
  {
    OpenFDMAssert((norm(mNormal)-1) < 8*Limits<value_type>::epsilon());
  }
  Plane(const Vector3& normal, const Vector3& off)
    : mNormal(normal), mDist(-dot(normal, off))
  {
    OpenFDMAssert((norm(mNormal)-1) < 8*Limits<value_type>::epsilon());
  }

  const Vector3& getNormal(void) const
  { return mNormal; }
  void setNormal(const Vector3& normal)
  {
    mNormal = normal;
    OpenFDMAssert((norm(mNormal)-1) < 8*Limits<value_type>::epsilon());
  }

  value_type getDist(void) const
  { return mDist; }
  void setDist(value_type dist)
  { mDist = dist; }

  /** Distance of a point from an infinite plane.
      Computes and returns the distance of a point from an infinite plane.
      Positive distance means that the point is at the same side like the
      normal vector.
   */
  value_type getDist(const Vector3& pt) const
  { return dot(mNormal, pt) + mDist; }

  Vector3 getPointOnPlane(void) const
  { return (-mDist)*mNormal; }

  /** Set plane from normal and point in plane.
      Sets the plane to a plane with the normal given in the argument
      and such that the point given on off is part of the plane.
   */
  void setNormalOff(const Vector3& normal, const Vector3& off)
  {
    mNormal = normal;
    mDist = - dot(normal, off);
  }

  value_type scalarProjectToNormal(const Vector3& v) const
  { return dot(mNormal, v); }
  Vector3 projectToNormal(const Vector3& v) const
  { return scalarProjectToNormal(v)*mNormal; }
  Vector3 projectToPlane(const Vector3& v) const
  { return v - projectToNormal(v); }

  bool intersectLine(const Vector3& base, const Vector3& dir, Vector3& ip) const
  {
    Vector3 ndir = normalize(dir);
    value_type tmp = dot(ndir, mNormal);

    // Check if the line is parallel to the plane.
    if (abs(tmp) < Limits<value_type>::epsilon())
      return false;

    ip = base - (getDist(base)/tmp)*ndir;
    return true;
  }
  
private:
  Vector3 mNormal;
  value_type mDist;
};

} // namespace OpenFDM

#endif
