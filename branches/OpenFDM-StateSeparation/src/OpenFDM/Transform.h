/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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
OpenFDM_FORCE_INLINE Vector3
posTo(const Vector3& p, const Rotation& r, const Vector3& v)
{
  return r.transform(v - p);
}
OpenFDM_FORCE_INLINE Vector3
posTo(const Vector3& p, const Vector3& v)
{
  return v - p;
}
OpenFDM_FORCE_INLINE Vector3
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
OpenFDM_FORCE_INLINE Vector3
posFrom(const Vector3& p, const Rotation& r, const Vector3& v)
{
  return r.backTransform(v) + p;
}
OpenFDM_FORCE_INLINE Vector3
posFrom(const Vector3& p, const Vector3& v)
{
  return v + p;
}
OpenFDM_FORCE_INLINE Vector3
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
OpenFDM_FORCE_INLINE Vector6
motionTo(const Vector3& p, const Rotation& r, const Vector6& v)
{
  Vector3 tv1 = v.getAngular();
  Vector3 tv2 = v.getLinear();
  tv2 -= cross(p, tv1);
  return Vector6(r.transform(tv1), r.transform(tv2));
}
OpenFDM_FORCE_INLINE Vector6
motionTo(const Vector3& p, const Vector6& v)
{
  Vector3 tv1 = v.getAngular();
  Vector3 tv2 = v.getLinear();
  tv2 -= cross(p, tv1);
  return Vector6(tv1, tv2);
}
OpenFDM_FORCE_INLINE Vector6
motionTo(const Rotation& r, const Vector6& v)
{
  Vector3 tv1 = v.getAngular();
  Vector3 tv2 = v.getLinear();
  return Vector6(r.transform(tv1), r.transform(tv2));
}

OpenFDM_FORCE_INLINE Vector6
angularMotionTo(const Vector3& p, const Rotation& r, const Vector3& v)
{
  return Vector6(r.transform(cross(v, p)), Vector3::zeros());
}
OpenFDM_FORCE_INLINE Vector6
angularMotionTo(const Rotation& r, const Vector3& v)
{
  return Vector6(r.transform(v), Vector3::zeros());
}
OpenFDM_FORCE_INLINE Vector6
angularMotionTo(const Vector3& p, const Vector3& v)
{
  return Vector6(cross(v, p), Vector3::zeros());
}

/** Spatial motion vector transform.
    Transforms a spatial motion vector from the current frame to the parent
    frame.
    @param v The motion vector in the current frame to be transformed.
    @return  The motion vector transformed to the parent frame.
*/
OpenFDM_FORCE_INLINE Vector6
motionFrom(const Vector3& p, const Rotation& r, const Vector6& v)
{
  Vector3 tv1 = r.backTransform(v.getAngular());
  Vector3 tv2 = r.backTransform(v.getLinear());
  tv2 += cross(p, tv1);
  return Vector6(tv1, tv2);
}
OpenFDM_FORCE_INLINE Vector6
motionFrom(const Vector3& p, const Vector6& v)
{
  Vector3 tv1 = v.getAngular();
  Vector3 tv2 = v.getLinear();
  tv2 += cross(p, tv1);
  return Vector6(tv1, tv2);
}
OpenFDM_FORCE_INLINE Vector6
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
OpenFDM_FORCE_INLINE Vector6
forceTo(const Vector3& p, const Rotation& r, const Vector6& f)
{
  Vector3 tf1 = f.getAngular();
  Vector3 tf2 = f.getLinear();
  tf1 -= cross(p, tf2);
  return Vector6(r.transform(tf1), r.transform(tf2));
}
OpenFDM_FORCE_INLINE Vector6
forceTo(const Vector3& p, const Vector6& f)
{
  Vector3 tf1 = f.getAngular();
  Vector3 tf2 = f.getLinear();
  tf1 -= cross(p, tf2);
  return Vector6(tf1, tf2);
}
OpenFDM_FORCE_INLINE Vector6
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
OpenFDM_FORCE_INLINE Vector6
forceFrom(const Vector3& p, const Rotation& r, const Vector6& f)
{
  Vector3 tf1 = r.backTransform(f.getAngular());
  Vector3 tf2 = r.backTransform(f.getLinear());
  tf1 += cross(p, tf2);
  return Vector6(tf1, tf2);
}
OpenFDM_FORCE_INLINE Vector6
forceFrom(const Vector3& p, const Vector6& f)
{
  Vector3 tf1 = f.getAngular();
  Vector3 tf2 = f.getLinear();
  tf1 += cross(p, tf2);
  return Vector6(tf1, tf2);
}
OpenFDM_FORCE_INLINE Vector6
forceFrom(const Rotation& r, const Vector6& f)
{
  Vector3 tf1 = r.backTransform(f.getAngular());
  Vector3 tf2 = r.backTransform(f.getLinear());
  return Vector6(tf1, tf2);
}
OpenFDM_FORCE_INLINE Vector6
forceFrom(const Vector3& p, const Rotation& r, const Vector3& f)
{
  Vector3 tf2 = r.backTransform(f);
  return Vector6(cross(p, tf2), tf2);
}
OpenFDM_FORCE_INLINE Vector6
forceFrom(const Vector3& p, const Vector3& f)
{
  return Vector6(cross(p, f), f);
}
OpenFDM_FORCE_INLINE Vector6
forceFrom(const Rotation& r, const Vector3& f)
{
  return Vector6(Vector3::zeros(), r.backTransform(f));
}

/**
 */
OpenFDM_FORCE_INLINE Plane
planeTo(const Vector3& p, const Rotation& r, const Plane& plane)
{
  return Plane(r.transform(plane.getNormal()),
               plane.getDist() + dot(plane.getNormal(), p));
}
OpenFDM_FORCE_INLINE Plane
planeTo(const Vector3& p, const Plane& plane)
{
  return Plane(plane.getNormal(), plane.getDist() + dot(plane.getNormal(), p));
}
OpenFDM_FORCE_INLINE Plane
planeTo(const Rotation& r, const Plane& plane)
{
  return Plane(r.transform(plane.getNormal()), plane.getDist());
}


/**
 */
OpenFDM_FORCE_INLINE Plane
planeFrom(const Vector3& p, const Rotation& r, const Plane& plane)
{
  return Plane(r.backTransform(plane.getNormal()),
               plane.getDist() - dot(plane.getNormal(), p));
}
OpenFDM_FORCE_INLINE Plane
planeFrom(const Vector3& p, const Plane& plane)
{
  return Plane(plane.getNormal(), plane.getDist() - dot(plane.getNormal(), p));
}
OpenFDM_FORCE_INLINE Plane
planeFrom(const Rotation& r, const Plane& plane)
{
  return Plane(r.backTransform(plane.getNormal()), plane.getDist());
}



OpenFDM_FORCE_INLINE SpatialInertia
inertiaFrom(const Vector3& p, const Rotation& r, const SpatialInertia& I)
{
  InertiaMatrix I11(I(0,0), I(1,0), I(2,0), I(1,1), I(2,1), I(2,2));
  InertiaMatrix I22(I(3,3), I(4,3), I(5,3), I(4,4), I(5,4), I(5,5));
  
  Matrix33 RI11(trans(r.getTransform())*I11);
  Matrix33 RI22(trans(r.getTransform())*I22);
  
  InertiaMatrix RI11R(RI11*r.getTransform());
  InertiaMatrix RI22R(RI22*r.getTransform());
  
  SpatialInertia It2;
  It2(3,3) = RI22R(0,0);
  It2(4,3) = RI22R(1,0);
  It2(5,3) = RI22R(2,0);
  It2(4,4) = RI22R(1,1);
  It2(5,4) = RI22R(2,1);
  It2(5,5) = RI22R(2,2);
  
  Matrix33 I21(I(3,0), I(3,1), I(3,2),
               I(4,0), I(4,1), I(4,2),
               I(5,0), I(5,1), I(5,2));
  Matrix33 RI21(trans(r.getTransform())*I21);
  Matrix33 RI21R(RI21*r.getTransform());
  
  Matrix33 pRI22R(cross(RI22R, p));
  
  Matrix33 I21new = RI21R - pRI22R;
  It2(3,0) = I21new(0,0);
  It2(4,0) = I21new(1,0);
  It2(5,0) = I21new(2,0);
  It2(3,1) = I21new(0,1);
  It2(4,1) = I21new(1,1);
  It2(5,1) = I21new(2,1);
  It2(3,2) = I21new(0,2);
  It2(4,2) = I21new(1,2);
  It2(5,2) = I21new(2,2);
  
  InertiaMatrix pRI22Rp(cross(pRI22R, p));
  RI11R -= pRI22Rp;
  Matrix33 pRI21R(cross(p, RI21R));
  RI11R += InertiaMatrix(pRI21R);
  RI11R += InertiaMatrix(trans(pRI21R));
  
  It2(0,0) = RI11R(0,0);
  It2(1,0) = RI11R(1,0);
  It2(2,0) = RI11R(2,0);
  It2(1,1) = RI11R(1,1);
  It2(2,1) = RI11R(2,1);
  It2(2,2) = RI11R(2,2);
  
  return It2;
}
OpenFDM_FORCE_INLINE SpatialInertia
inertiaFrom(const Vector3& p, const SpatialInertia& I)
{
  InertiaMatrix I11(I(0,0), I(1,0), I(2,0), I(1,1), I(2,1), I(2,2));
  InertiaMatrix I22(I(3,3), I(4,3), I(5,3), I(4,4), I(5,4), I(5,5));
  
  SpatialInertia It2;
  It2(3,3) = I22(0,0);
  It2(4,3) = I22(1,0);
  It2(5,3) = I22(2,0);
  It2(4,4) = I22(1,1);
  It2(5,4) = I22(2,1);
  It2(5,5) = I22(2,2);
  
  Matrix33 I21(I(3,0), I(3,1), I(3,2),
               I(4,0), I(4,1), I(4,2),
               I(5,0), I(5,1), I(5,2));
  
  Matrix33 pI22(cross(I22, p));
  
  Matrix33 I21new = I21 - pI22;
  It2(3,0) = I21new(0,0);
  It2(4,0) = I21new(1,0);
  It2(5,0) = I21new(2,0);
  It2(3,1) = I21new(0,1);
  It2(4,1) = I21new(1,1);
  It2(5,1) = I21new(2,1);
  It2(3,2) = I21new(0,2);
  It2(4,2) = I21new(1,2);
  It2(5,2) = I21new(2,2);
  
  InertiaMatrix pI22p(cross(pI22, p));
  
  I11 -= pI22p;
  Matrix33 pI21(cross(p, I21));
  I11 += InertiaMatrix(pI21);
  I11 += InertiaMatrix(trans(pI21));
  
  It2(0,0) = I11(0,0);
  It2(1,0) = I11(1,0);
  It2(2,0) = I11(2,0);
  It2(1,1) = I11(1,1);
  It2(2,1) = I11(2,1);
  It2(2,2) = I11(2,2);
  
  return It2;
}
OpenFDM_FORCE_INLINE SpatialInertia
inertiaFrom(const Rotation& r, const SpatialInertia& I)
{
  InertiaMatrix I11(I(0,0), I(1,0), I(2,0), I(1,1), I(2,1), I(2,2));
  InertiaMatrix I22(I(3,3), I(4,3), I(5,3), I(4,4), I(5,4), I(5,5));
  
  Matrix33 RI11(trans(r.getTransform())*I11);
  Matrix33 RI22(trans(r.getTransform())*I22);
  
  InertiaMatrix RI11R(RI11*r.getTransform());
  InertiaMatrix RI22R(RI22*r.getTransform());
  
  SpatialInertia It2;
  It2(3,3) = RI22R(0,0);
  It2(4,3) = RI22R(1,0);
  It2(5,3) = RI22R(2,0);
  It2(4,4) = RI22R(1,1);
  It2(5,4) = RI22R(2,1);
  It2(5,5) = RI22R(2,2);
  
  Matrix33 I21(I(3,0), I(3,1), I(3,2),
               I(4,0), I(4,1), I(4,2),
               I(5,0), I(5,1), I(5,2));
  Matrix33 RI21(trans(r.getTransform())*I21);
  Matrix33 RI21R(RI21*r.getTransform());
  
  Matrix33 I21new = RI21R;
  It2(3,0) = I21new(0,0);
  It2(4,0) = I21new(1,0);
  It2(5,0) = I21new(2,0);
  It2(3,1) = I21new(0,1);
  It2(4,1) = I21new(1,1);
  It2(5,1) = I21new(2,1);
  It2(3,2) = I21new(0,2);
  It2(4,2) = I21new(1,2);
  It2(5,2) = I21new(2,2);
  
  It2(0,0) = RI11R(0,0);
  It2(1,0) = RI11R(1,0);
  It2(2,0) = RI11R(2,0);
  It2(1,1) = RI11R(1,1);
  It2(2,1) = RI11R(2,1);
  It2(2,2) = RI11R(2,2);
  
  return It2;
}

} // namespace OpenFDM

#endif
