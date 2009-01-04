/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Frame_H
#define OpenFDM_Frame_H

#include "Assert.h"
#include "Vector.h"
#include "Plane.h"
#include "Transform.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Rotation.h"

namespace OpenFDM {

/** 
The \ref Frame class is the basic tool to model a tree of moving and
accelerating coordinate frames.
*/

class Frame {
public:
  Frame() :
    mPosition(Vector3::zeros()),
    mOrientation(Quaternion::unit()),
    mRelVel(Vector6::zeros()),
    mRelVelDot(Vector6::zeros()),
    mParentSpVel(Vector6::zeros()),
    mParentSpAccel(Vector6::zeros()),
    mRefOrient(Quaternion::unit()),
    mRefPos(Vector3::zeros()),
    mRefVel(Vector6::zeros())
  { }
  ~Frame(void)
  { }

  void setPosAndVel(const Frame& parent)
  {
    mPosition = Vector3::zeros();
    mOrientation = Quaternion::unit();
    mRelVel = Vector6::zeros();

    mRefOrient = parent.getRefOrientation();
    mRefPos = parent.getRefPosition();
    mRefVel = parent.getRefVel();

    mParentSpVel = parent.getSpVel();
  }

  void setAccel(const Frame& parent)
  {
    mRelVelDot = Vector6::zeros();
    mParentSpAccel = parent.getSpAccel();
  }

  void setPosAndVel(const Frame& parent, const Vector3& position,
                    const Quaternion& orientation, const Vector6& velocity)
  {
    mPosition = position;
    mOrientation = orientation;
    mRelVel = velocity;

    mRefOrient = parent.getRefOrientation()*getOrientation();
    mRefPos = parent.posToRef(getPosition());
    mRefVel = velocity + motionFromParent(parent.getRefVel());

    mParentSpVel = motionFromParent(parent.getSpVel());
  }

  void setAccel(const Frame& parent, const Vector6& acceleration)
  {
    mRelVelDot = acceleration;
    mParentSpAccel = motionFromParent(parent.getSpAccel());
  }

  void setPosAndVel(const Vector3& parentAngularVel, const Vector3& position,
                    const Quaternion& orientation, const Vector6& velocity)
  {
    mPosition = position;
    mOrientation = orientation;
    mRelVel = velocity;

    mRefOrient = orientation;
    mRefPos = position;

    mParentSpVel = angularMotionTo(mPosition, mOrientation, parentAngularVel);
    mRefVel = velocity + mParentSpVel;
  }

  void setAccel(const Vector6& acceleration)
  {
    mRelVelDot = acceleration;
    mParentSpAccel = Vector6::zeros();
  }

  void setSpAccel(const Vector6& spatialAcceleration)
  {
    mParentSpAccel = Vector6::zeros();
    mRelVelDot = spatialAcceleration - mParentSpAccel - getHdot();
  }
  void setSpAccel(const Frame& parent, const Vector6& spatialAcceleration)
  {
    mParentSpAccel = motionFromParent(parent.getSpAccel());
    mRelVelDot = spatialAcceleration - mParentSpAccel - getHdot();
  }

  /** Position of the current frame.
      @return The position vector of the current frame with repsect to the
      parent frame. It is the only vector which is in the parent frames
      coordinates.
   */
  const Vector3& getPosition(void) const
  { return mPosition; }

  /** Orientation of the current frame.
      @return The orientation of the current frame with repsect to the
      parent frame. The quaternion returned here rotates vectors in the parent
      frames coordinated to vectors in current frames coordinates.
   */
  const Rotation& getOrientation(void) const
  { return mOrientation; }


  /** Rotation to parent frames coordinates.
      @param v Vector in current frames coordinates to rotate to parent
               frames coordinates.
      @return  The vector v in the parent frames coordinates.
   */
  Vector3 rotToParent(const Vector3& v) const
  { return getOrientation().backTransform(v); }

  /** Rotation to current frames coordinates.
      @param v Vector in parent frames coordinates to rotate to current
               frames coordinates.
      @return  The vector v in the current frames coordinates.
   */
  Vector3 rotFromParent(const Vector3& v) const
  { return getOrientation().transform(v); }


  /** Relative spatial velocity.
      @return The spatial velocity vector of the current frame with respect
              to the parent frame. The velocity is in the current frames
              coordinates.
   */
  const Vector6& getRelVel(void) const
  { return mRelVel; }

  /** Relative spatial acceleration.
      @return The spatial acceleration vector of the current frame with respect
              to the parent frame. The velocity is in the current frames
              coordinates.
   */
  const Vector6& getRelVelDot(void) const 
  { return mRelVelDot; }


  /** Linear velocity with respect to parent.
      @return The linear velocity of this frame with respect to the parent
      frame.
   */
  Vector3 getLinearRelVel(void) const
  { return getRelVel().getLinear(); }

  /** Angular velocity with respect to parent.
      @return The angular velocity of this frame with respect to the parent
      frame.
   */
  Vector3 getAngularRelVel(void) const
  { return getRelVel().getAngular(); }

  /** Spatial velocity of the parent frame.
      @return The spatial velocity of the parent frame with respect to an
      inertial frame transformed to the current frame. If the current frame
      does not have a parent it is assumed to be an inertial frame.
   */
  const Vector6& getParentSpVel(void) const
  { return mParentSpVel; }

  /** Spatial velocity of the current frame.
      @return The spatial velocity of the current frame with respect to an
      inertial frame. If the current frame does not have a parent it is
      assumed to be an inertial frame.
   */
  Vector6 getSpVel(void) const
  { return getRelVel() + getParentSpVel(); }

  const Vector6& getRefVel(void) const
  { return mRefVel; }

  /** Linear acceleration with respect to parent.
      @return The linear acceleration of this frame with respect to the parent
      frame.
   */
  Vector3 getLinearRelVelDot(void) const
  { return getRelVelDot().getLinear(); }

  /** Angular acceleration with respect to parent.
      @return The angular acceleration of this frame with respect to the parent
      frame.
   */
  Vector3 getAngularRelVelDot(void) const
  { return getRelVelDot().getAngular(); }

  /** Spatial acceleration of the parent frame.
      @return The spatial acceleration of the parent frame with respect to an
      inertial frame transformed to the current frame. If the current frame
      does not have a parent it is assumed to be an inertial frame.
      Note that the spatial acceleration is not the classical acceleration
      of the moving and accelerating body (@see getClassicAccel).
   */
  const Vector6& getParentSpAccel(void) const
  { return mParentSpAccel; }

  /** Spatial acceleration of the current frame.
      @return The spatial acceleration of the current frame with respect to an
      inertial frame. If the current frame does not have a parent it is
      assumed to be an inertial frame.
      Note that the spatial acceleration is not the classical acceleration
      of the moving and accelerating body (@see getClassicAccel).
   */
  Vector6 getSpAccel(void) const
  { return getRelVelDot() + getParentSpAccel() + getHdot(); }

  /** Classical acceleration of the current frame.
      @return The sensed acceleration of the current frame with respect to an
      inertial frame. If the current frame does not have a parent it is
      assumed to be an inertial frame.
   */
  Vector6 getClassicAccel(void) const
  {
    Vector6 iv = getSpVel();
    return getRelVelDot() + getParentSpAccel() + getHdot()
      + Vector6(Vector3::zeros(), cross(iv.getAngular(), iv.getLinear()));
  }


  /** FIXME belongs into the joints.
   */
  Vector6 getHdot(void) const
  {
    /**
       This is the cross product of the inertial spatial velocity
       vector with the relative spatial velocity vector (motion type
       cross product). Since the inertial velocity is the transformed
       inertial velocity of the parent frame plus the relative
       velocity of the current frame, all the relative velocity
       components cancel out in this expression. What remains is the
       transformed spatial velocity of the parent frame cross the
       relative velocity.
     */
    Vector6 pivel = getParentSpVel();
    return Vector6(cross(pivel.getAngular(), getAngularRelVel()),
                   cross(pivel.getAngular(), getLinearRelVel()) + 
                   cross(pivel.getLinear(), getAngularRelVel()));
  }

  /** Position vector transform.
      Transforms a position vector from the parent frame to the current
      frame.
      @param v The position vector in the parent frame to be transformed.
      @return  The motion vector transformed to the current frame.
   */
  Vector3 posFromParent(const Vector3& v) const
  { return posTo(getPosition(), getOrientation(), v); }

  /** Position vector transform.
      Transforms a position vector from the current frame to the parent
      frame.
      @param v The position in the current frame to be transformed.
      @return  The position transformed to the parent frame.
   */
  Vector3 posToParent(const Vector3& v) const
  { return posFrom(getPosition(), getOrientation(), v); }

  /** Spatial motion vector transform.
      Transforms a spatial motion vector from the parent frame to the current
      frame.
      @param v The motion vector in the parent frame to be transformed.
      @return  The motion vector transformed to the current frame.
   */
  Vector6 motionFromParent(const Vector6& v) const
  { return motionTo(getPosition(), getOrientation(), v); }

  /** Spatial motion vector transform.
      Transforms a spatial motion vector from the current frame to the parent
      frame.
      @param v The motion vector in the current frame to be transformed.
      @return  The motion vector transformed to the parent frame.
   */
  Vector6 motionToParent(const Vector6& v) const
  { return motionFrom(getPosition(), getOrientation(), v); }

  /** Spatial force vector transform.
      Transforms a spatial force vector from the parent frame to the current
      frame.
      @param v The force vector in the parent frame to be transformed.
      @return  The force vector transformed to the current frame.
   */
  Vector6 forceFromParent(const Vector6& v) const
  { return forceTo(getPosition(), getOrientation(), v); }

  /** Spatial force vector transform.
      Transforms a spatial force vector from the current frame to the parent
      frame.
      @param v The force vector in the current frame to be transformed.
      @return  The force vector transformed to the parent frame.
   */
  Vector6 forceToParent(const Vector6& v) const
  { return forceFrom(getPosition(), getOrientation(), v); }

  Plane planeFromParent(const Plane& plane) const
  { return planeTo(getPosition(), getOrientation(), plane); }

  Plane planeToParent(const Plane& plane) const
  { return planeFrom(getPosition(), getOrientation(), plane); }

  /** Rotation to reference frames coordinates.
      @param v Vector in current frames coordinates to rotate to reference
               frames coordinates.
      @return  The vector v in the reference frames coordinates.
   */
  Vector3 rotToRef(const Vector3& v) const
  { return getRefOrientation().backTransform(v); }

  /** Rotation from reference frames coordinates.
      @param v Vector in reference frames coordinates to rotate to current
               frames coordinates.
      @return  The vector v in the current frames coordinates.
   */
  Vector3 rotFromRef(const Vector3& v) const
  { return getRefOrientation().transform(v); }


  /** Position vector transform.
      Transforms a position vector from the reference frame to the current
      frame.
      @param v The position vector in the reference frame to be transformed.
      @return  The motion vector transformed to the current frame.
   */
  Vector3 posFromRef(const Vector3& v) const
  { return posTo(getRefPosition(), getRefOrientation(), v); }

  /** Position vector transform.
      Transforms a position vector from the current frame to the reference
      frame.
      @param v The position in the current frame to be transformed.
      @return  The position transformed to the reference frame.
   */
  Vector3 posToRef(const Vector3& v) const
  { return posFrom(getRefPosition(), getRefOrientation(), v); }


  /** Spatial motion vector transform.
      Transforms a spatial motion vector from the parent frame to the current
      frame.
      @param v The motion vector in the parent frame to be transformed.
      @return  The motion vector transformed to the current frame.
   */
  Vector6 motionFromRef(const Vector6& v) const
  { return motionTo(getRefPosition(), getRefOrientation(), v); }

  /** Spatial motion vector transform.
      Transforms a spatial motion vector from the current frame to the parent
      frame.
      @param v The motion vector in the current frame to be transformed.
      @return  The motion vector transformed to the parent frame.
   */
  Vector6 motionToRef(const Vector6& v) const
  { return motionFrom(getRefPosition(), getRefOrientation(), v); }

  Plane planeFromRef(const Plane& plane) const
  { return planeTo(getRefPosition(), getRefOrientation(), plane); }

  Plane planeToRef(const Plane& plane) const
  { return planeFrom(getRefPosition(), getRefOrientation(), plane); }

  /** Reference orientation.
   * Returns the reference orientation of this frame wrt the topmost frame 
   * this frame is attached to.
   * It is measured in the topmost frames coordinates.
   */
  const Quaternion& getRefOrientation(void) const
  { return mRefOrient; }

  /** Reference position.
   * Returns the reference position of this frame wrt the topmost frame 
   * this frame is attached to.
   * It is measured in the topmost frames coordinates.
   */
  const Vector3& getRefPosition(void) const
  { return mRefPos; }

private:
  // The offset of this frames origin wrt the parent frame measured in
  // the parent frames coordinates.
  Vector3 mPosition;
  // The orientation wrt the parent frame (measured in the parent frames
  Rotation mOrientation;

  // The spatial velocity wrt parent frame measured in
  // this frames coordinates.
  // True? more the relative velocity ...
  Vector6 mRelVel;

  Vector6 mRelVelDot;

  Vector6 mParentSpVel;
  Vector6 mParentSpAccel;

  Quaternion mRefOrient;
  Vector3 mRefPos;
  Vector6 mRefVel;
};

} // namespace OpenFDM

#endif
