/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Transform_H
#define OpenFDM_Transform_H

#include "Assert.h"
#include "Vector.h"
#include "Plane.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Rotation.h"
#include "Inertia.h"

namespace OpenFDM {

/** Position vector transform.
    Transforms a position vector from the current frame to a frame located at
    the position p and rotated with the orientation r.
    @param p The position vector of the child frame.
    @param r The orientation of the child frame.
    @param v The position vector in the current frame to be transformed.
    @return  The position vector transformed to the child frame.
*/
inline Vector3
posTo(const Vector3& p, const Rotation& r, const Vector3& v)
{
  return r.transform(v - p);
}
inline Vector3
posTo(const Vector3& p, const Vector3& v)
{
  return v - p;
}
inline Vector3
posTo(const Rotation& r, const Vector3& v)
{
  return r.transform(v);
}


/** Position vector transform.
    Transforms a position vector from the current frame to a frame located at
    the position p and rotated with the orientation r.
    @param p The position vector of the child frame.
    @param r The orientation of the child frame.
    @param v The position vector in the child frame to be transformed.
    @return  The position vector transformed to the current frame.
*/
inline Vector3
posFrom(const Vector3& p, const Rotation& r, const Vector3& v)
{
  return r.backTransform(v) + p;
}
inline Vector3
posFrom(const Vector3& p, const Vector3& v)
{
  return v + p;
}
inline Vector3
posFrom(const Rotation& r, const Vector3& v)
{
  return r.backTransform(v);
}


/** Spatial motion vector transform.
    Transforms a spatial motion vector from the parent frame to the current
    frame.
    @param v The motion vector in the parent frame to be transformed.
    @return  The motion vector transformed to the current frame.
*/
inline Vector6
motionTo(const Vector3& p, const Rotation& r, const Vector6& v)
{
  Vector3 tv1 = v.getAngular();
  Vector3 tv2 = v.getLinear();
  tv2 -= cross(p, tv1);
  return Vector6(r.transform(tv1), r.transform(tv2));
}
inline Vector6
motionTo(const Vector3& p, const Vector6& v)
{
  Vector3 tv1 = v.getAngular();
  Vector3 tv2 = v.getLinear();
  tv2 -= cross(p, tv1);
  return Vector6(tv1, tv2);
}
inline Vector6
motionTo(const Rotation& r, const Vector6& v)
{
  Vector3 tv1 = v.getAngular();
  Vector3 tv2 = v.getLinear();
  return Vector6(r.transform(tv1), r.transform(tv2));
}

/** Spatial motion vector transform.
    Transforms a spatial motion vector from the current frame to the parent
    frame.
    @param v The motion vector in the current frame to be transformed.
    @return  The motion vector transformed to the parent frame.
*/
inline Vector6
motionFrom(const Vector3& p, const Rotation& r, const Vector6& v)
{
  Vector3 tv1 = r.backTransform(v.getAngular());
  Vector3 tv2 = r.backTransform(v.getLinear());
  tv2 += cross(p, tv1);
  return Vector6(tv1, tv2);
}
inline Vector6
motionFrom(const Vector3& p, const Vector6& v)
{
  Vector3 tv1 = v.getAngular();
  Vector3 tv2 = v.getLinear();
  tv2 += cross(p, tv1);
  return Vector6(tv1, tv2);
}
inline Vector6
motionFrom(const Rotation& r, const Vector6& v)
{
  Vector3 tv1 = r.backTransform(v.getAngular());
  Vector3 tv2 = r.backTransform(v.getLinear());
  return Vector6(tv1, tv2);
}


/** Spatial force vector transform.
    Transforms a spatial force vector from the parent frame to the current
    frame.
    @param v The force vector in the parent frame to be transformed.
    @return  The force vector transformed to the current frame.
*/
inline Vector6
forceTo(const Vector3& p, const Rotation& r, const Vector6& f)
{
  Vector3 tf1 = f.getAngular();
  Vector3 tf2 = f.getLinear();
  tf1 -= cross(p, tf2);
  return Vector6(r.transform(tf1), r.transform(tf2));
}
inline Vector6
forceTo(const Vector3& p, const Vector6& f)
{
  Vector3 tf1 = f.getAngular();
  Vector3 tf2 = f.getLinear();
  tf1 -= cross(p, tf2);
  return Vector6(tf1, tf2);
}
inline Vector6
forceTo(const Rotation& r, const Vector6& f)
{
  Vector3 tf1 = f.getAngular();
  Vector3 tf2 = f.getLinear();
  return Vector6(r.transform(tf1), r.transform(tf2));
}

/** Spatial force vector transform.
    Transforms a spatial force vector from the current frame to the parent
    frame.
    @param v The force vector in the current frame to be transformed.
    @return  The force vector transformed to the parent frame.
*/
inline Vector6
forceFrom(const Vector3& p, const Rotation& r, const Vector6& f)
{
  Vector3 tf1 = r.backTransform(f.getAngular());
  Vector3 tf2 = r.backTransform(f.getLinear());
  tf1 += cross(p, tf2);
  return Vector6(tf1, tf2);
}
inline Vector6
forceFrom(const Vector3& p, const Vector6& f)
{
  Vector3 tf1 = f.getAngular();
  Vector3 tf2 = f.getLinear();
  tf1 += cross(p, tf2);
  return Vector6(tf1, tf2);
}
inline Vector6
forceFrom(const Rotation& r, const Vector6& f)
{
  Vector3 tf1 = r.backTransform(f.getAngular());
  Vector3 tf2 = r.backTransform(f.getLinear());
  return Vector6(tf1, tf2);
}
inline Vector6
forceFrom(const Vector3& p, const Rotation& r, const Vector3& f)
{
  Vector3 tf2 = r.backTransform(f);
  return Vector6(cross(p, tf2), tf2);
}
inline Vector6
forceFrom(const Vector3& p, const Vector3& f)
{
  return Vector6(cross(p, f), f);
}
inline Vector6
forceFrom(const Rotation& r, const Vector3& f)
{
  return Vector6(Vector3::zeros(), r.backTransform(f));
}

/**
 */
inline Plane
planeTo(const Vector3& p, const Rotation& r, const Plane& plane)
{
  // FIXME: simplify
  return Plane(normalize(r.transform(plane.getNormal())),
               posTo(p, r, plane.getPointOnPlane()));
}
inline Plane
planeTo(const Vector3& p, const Plane& plane)
{
  // FIXME: simplify
  return Plane(plane.getNormal(),
               posTo(p, plane.getPointOnPlane()));
}
inline Plane
planeTo(const Rotation& r, const Plane& plane)
{
  // FIXME: simplify
  return Plane(normalize(r.transform(plane.getNormal())),
               posTo(r, plane.getPointOnPlane()));
}


/**
 */
inline Plane
planeFrom(const Vector3& p, const Rotation& r, const Plane& plane)
{
  // FIXME: simplify
  return Plane(normalize(r.backTransform(plane.getNormal())),
               posFrom(p, r, plane.getPointOnPlane()));
}
inline Plane
planeFrom(const Vector3& p, const Plane& plane)
{
  // FIXME: simplify
  return Plane(plane.getNormal(),
               posFrom(p, plane.getPointOnPlane()));
}
inline Plane
planeFrom(const Rotation& r, const Plane& plane)
{
  // FIXME: simplify
  return Plane(normalize(r.backTransform(plane.getNormal())),
               posFrom(r, plane.getPointOnPlane()));
}



inline SpatialInertia
inertiaFrom(const Vector3& p, const Rotation& r, const SpatialInertia& I)
{
  InertiaMatrix I11(I(1,1),
                    I(2,1), I(2,2),
                    I(3,1), I(3,2), I(3,3));
  InertiaMatrix I22(I(4,4),
                    I(5,4), I(5,5),
                    I(6,4), I(6,5), I(6,6));
  
  Matrix33 RI11(trans(r.getTransform())*I11);
  Matrix33 RI22(trans(r.getTransform())*I22);
  
  InertiaMatrix RI11R(RI11*r.getTransform());
  InertiaMatrix RI22R(RI22*r.getTransform());
  
  SpatialInertia It2;
  It2(4,4) = RI22R(1,1);
  It2(5,4) = RI22R(2,1);
  It2(6,4) = RI22R(3,1);
  It2(5,5) = RI22R(2,2);
  It2(6,5) = RI22R(3,2);
  It2(6,6) = RI22R(3,3);
  
  Matrix33 I21(I(4,1), I(4,2), I(4,3),
               I(5,1), I(5,2), I(5,3),
               I(6,1), I(6,2), I(6,3));
  Matrix33 RI21(trans(r.getTransform())*I21);
  Matrix33 RI21R(RI21*r.getTransform());
  
  Matrix33 pRI22R(cross(RI22R, p));
  
  Matrix33 I21new = RI21R - pRI22R;
  It2(4,1) = I21new(1,1);
  It2(5,1) = I21new(2,1);
  It2(6,1) = I21new(3,1);
  It2(4,2) = I21new(1,2);
  It2(5,2) = I21new(2,2);
  It2(6,2) = I21new(3,2);
  It2(4,3) = I21new(1,3);
  It2(5,3) = I21new(2,3);
  It2(6,3) = I21new(3,3);
  
  InertiaMatrix pRI22Rp(cross(pRI22R, p));
  RI11R -= pRI22Rp;
  Matrix33 pRI21R(cross(p, RI21R));
  RI11R += InertiaMatrix(pRI21R);
  RI11R += InertiaMatrix(trans(pRI21R));
  
  It2(1,1) = RI11R(1,1);
  It2(2,1) = RI11R(2,1);
  It2(3,1) = RI11R(3,1);
  It2(2,2) = RI11R(2,2);
  It2(3,2) = RI11R(3,2);
  It2(3,3) = RI11R(3,3);
  
  return It2;
}
inline SpatialInertia
inertiaFrom(const Vector3& p, const SpatialInertia& I)
{
  InertiaMatrix I11(I(1,1),
                    I(2,1), I(2,2),
                    I(3,1), I(3,2), I(3,3));
  InertiaMatrix I22(I(4,4),
                    I(5,4), I(5,5),
                    I(6,4), I(6,5), I(6,6));
  
  SpatialInertia It2;
  It2(4,4) = I22(1,1);
  It2(5,4) = I22(2,1);
  It2(6,4) = I22(3,1);
  It2(5,5) = I22(2,2);
  It2(6,5) = I22(3,2);
  It2(6,6) = I22(3,3);
  
  Matrix33 I21(I(4,1), I(4,2), I(4,3),
               I(5,1), I(5,2), I(5,3),
               I(6,1), I(6,2), I(6,3));
  
  Matrix33 pI22(cross(I22, p));
  
  Matrix33 I21new = I21 - pI22;
  It2(4,1) = I21new(1,1);
  It2(5,1) = I21new(2,1);
  It2(6,1) = I21new(3,1);
  It2(4,2) = I21new(1,2);
  It2(5,2) = I21new(2,2);
  It2(6,2) = I21new(3,2);
  It2(4,3) = I21new(1,3);
  It2(5,3) = I21new(2,3);
  It2(6,3) = I21new(3,3);
  
  InertiaMatrix pI22p(cross(pI22, p));
  
  I11 -= pI22p;
  Matrix33 pI21(cross(p, I21));
  I11 += InertiaMatrix(pI21);
  I11 += InertiaMatrix(trans(pI21));
  
  It2(1,1) = I11(1,1);
  It2(2,1) = I11(2,1);
  It2(3,1) = I11(3,1);
  It2(2,2) = I11(2,2);
  It2(3,2) = I11(3,2);
  It2(3,3) = I11(3,3);
  
  return It2;
}
inline SpatialInertia
inertiaFrom(const Rotation& r, const SpatialInertia& I)
{
  InertiaMatrix I11(I(1,1),
                    I(2,1), I(2,2),
                    I(3,1), I(3,2), I(3,3));
  InertiaMatrix I22(I(4,4),
                    I(5,4), I(5,5),
                    I(6,4), I(6,5), I(6,6));
  
  Matrix33 RI11(trans(r.getTransform())*I11);
  Matrix33 RI22(trans(r.getTransform())*I22);
  
  InertiaMatrix RI11R(RI11*r.getTransform());
  InertiaMatrix RI22R(RI22*r.getTransform());
  
  SpatialInertia It2;
  It2(4,4) = RI22R(1,1);
  It2(5,4) = RI22R(2,1);
  It2(6,4) = RI22R(3,1);
  It2(5,5) = RI22R(2,2);
  It2(6,5) = RI22R(3,2);
  It2(6,6) = RI22R(3,3);
  
  Matrix33 I21(I(4,1), I(4,2), I(4,3),
               I(5,1), I(5,2), I(5,3),
               I(6,1), I(6,2), I(6,3));
  Matrix33 RI21(trans(r.getTransform())*I21);
  Matrix33 RI21R(RI21*r.getTransform());
  
  Matrix33 I21new = RI21R;
  It2(4,1) = I21new(1,1);
  It2(5,1) = I21new(2,1);
  It2(6,1) = I21new(3,1);
  It2(4,2) = I21new(1,2);
  It2(5,2) = I21new(2,2);
  It2(6,2) = I21new(3,2);
  It2(4,3) = I21new(1,3);
  It2(5,3) = I21new(2,3);
  It2(6,3) = I21new(3,3);
  
  It2(1,1) = RI11R(1,1);
  It2(2,1) = RI11R(2,1);
  It2(3,1) = RI11R(3,1);
  It2(2,2) = RI11R(2,2);
  It2(3,2) = RI11R(3,2);
  It2(3,3) = RI11R(3,3);
  
  return It2;
}

} // namespace OpenFDM

#endif
