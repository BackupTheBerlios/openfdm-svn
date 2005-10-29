/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Frame_H
#define OpenFDM_Frame_H

#include <list>
#include <vector>
#include <string>

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Plane.h"
#include "Transform.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Rotation.h"
#include "Inertia.h"
#include "MultiBodyModel.h"

namespace OpenFDM {

class RigidBody;

/** 
The \ref Frame class is the basic tool to model a tree of moving and
accelerating coordinate frames.

Each frame can have one parent frame and several child frames. Each
frame can have a position and orientation offset with respect to its
parent frame. So, if a parent frame is relocated with respect to a
global coordinate system, all the child frames will be relocated with
respect to that global coordinate system too.

Each frame can have a linear and angular velocity with respect to its
parent frame.
*/

class Frame :
    public Object {
public:
  /// FIXME: store a 'path' to the current frame, check that by comparing the
  /// first path element
  typedef const void* frameid_type;

  /// Container for the child frames.
  typedef std::vector<shared_ptr<Frame> > ChildFrameList;

  Frame(const std::string& name);
  virtual ~Frame(void);

  /// Double dispatch helper for the multibody system visitor
  virtual void accept(Visitor& visitor);
  /// Double dispatch helper for the multibody system visitor
  virtual void traverse(Visitor& visitor);
  /// Double dispatch helper for the multibody system visitor
  virtual void accept(ConstVisitor& visitor) const;
  /// Double dispatch helper for the multibody system visitor
  virtual void traverse(ConstVisitor& visitor) const;

  /// Cast to a RigidBody Object if it is one.
  virtual RigidBody* toRigidBody(void);
  /// Cast to a const RigidBody Object if it is one.
  virtual const RigidBody* toRigidBody(void) const;

  /// Set the name of the Frame FIXME: move into Object ...
  void setName(const std::string& name)
  { mName = name; }
  /// The name of the Frame FIXME: move into Object ...
  const std::string& getName(void) const
  { return mName; }

  /// Return the parent frame.
  Frame* getParentFrame(void)
  { return mParentFrame; }
  /// Return the parent frame.
  const Frame* getParentFrame(void) const
  { return mParentFrame; }
  /// Return true if the given frame is the parent frame.
  bool isParentFrame(const Frame* frame) const
  { return frame == mParentFrame; }
  /// True if the current frame has a parent frame.
  bool hasParent(void) const
  { return getParentFrame(); }

  /// Adds the given frame to the list of child frames.
  /// returns true if that completed sucessfully.
  bool addChildFrame(Frame* child);
  /// Removes the given frame to the list of child frames.
  /// returns true if that completed sucessfully.
  bool removeChildFrame(Frame* child);
  /// Get the i-th child frame. Returns a 0 pointer if i is out of range.
  Frame* getChildFrame(unsigned i);
  /// Get the i-th child frame. Returns a 0 pointer if i is out of range.
  const Frame* getChildFrame(unsigned i) const;
  /// Returns the number of child frames
  unsigned getNumChildFrames(void) const
  { return mChildFrames.size(); }






  /////////////////////////////////////////
  typedef std::vector<shared_ptr<MultiBodyModel> > abchild_list;
  typedef abchild_list::iterator           abchild_iterator;
  typedef abchild_list::const_iterator     const_abchild_iterator;
  abchild_list _children;

  bool addMultiBodyModel(MultiBodyModel* child, unsigned parentNum = 0)
  {
    if (!child)
      return false;
    if (findMultiBodyModelIndex(child) < _children.size())
      return false;
    
    bool couldSetParent = child->setParentFrame(this, parentNum);
    if (couldSetParent)
      _children.push_back(child);
    return couldSetParent;
  }
  bool removeMultiBodyModel(const MultiBodyModel* child)
  { return removeMultiBodyModel(findMultiBodyModelIndex(child)); }
private:
  bool removeMultiBodyModel(unsigned i)
  {
    if (_children.size() <= i)
      return false;

    abchild_iterator it = _children.begin() + i;
    MultiBodyModel* child = *it;
    bool couldRemoveParent = child->removeParentFrame(this);
    if (couldRemoveParent)
      _children.erase(it);
    return couldRemoveParent;
  }
public:
  unsigned findMultiBodyModelIndex(const MultiBodyModel* child) const
  {
    unsigned i;
    for (i = 0; i < _children.size(); ++i) {
      if (_children[i] == child)
        return i;
    }
    return i;
  }

  MultiBodyModel* getMultiBodyModel(unsigned i)
  {
    if (_children.size() <= i)
      return 0;
    return _children[i];
  }
  const MultiBodyModel* getMultiBodyModel(unsigned i) const
  {
    if (_children.size() <= i)
      return 0;
    return _children[i];
  }

  unsigned getNumMultiBodyModels(void) const
  { return _children.size(); }

  //////////////////


  // Return the current frames frame id.
  frameid_type getFrameId(void) const
  { return this; }
  // Return the reference frames frame id.
  frameid_type getRefFrameId(void) const
  {
    if (mDirtyPos)
      computePositionDep();
    return mReferenceFrameId;
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
  Vector6 getRelAccel(void) const 
  { return mRelAccel; }


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
  {
    if (mDirtySpVel)
      computeVelocityDep();
    return mParentSpVel;
  }

  /** Spatial velocity of the current frame.
      @return The spatial velocity of the current frame with respect to an
      inertial frame. If the current frame does not have a parent it is
      assumed to be an inertial frame.
   */
  Vector6 getSpVel(void) const
  { return getRelVel() + getParentSpVel(); }

  Vector6 getRefVel(void) const
  {
    if (mDirtySpVel)
      computeVelocityDep();
    return mRefVel;
  }

  /** Linear acceleration with respect to parent.
      @return The linear acceleration of this frame with respect to the parent
      frame.
   */
  Vector3 getLinearRelAccel(void) const
  { return getRelAccel().getLinear(); }

  /** Angular acceleration with respect to parent.
      @return The angular acceleration of this frame with respect to the parent
      frame.
   */
  Vector3 getAngularRelAccel(void) const
  { return getRelAccel().getAngular(); }

  /** Spatial acceleration of the parent frame.
      @return The spatial acceleration of the parent frame with respect to an
      inertial frame transformed to the current frame. If the current frame
      does not have a parent it is assumed to be an inertial frame.
      Note that the spatial acceleration is not the classical acceleration
      of the moving and accelerating body (@see getClassicAccel).
   */
  const Vector6& getParentSpAccel(void) const
  {
    if (mDirtySpAccel)
      computeAccelerationDep();
    return mParentSpAccel;
  }

  /** Spatial acceleration of the current frame.
      @return The spatial acceleration of the current frame with respect to an
      inertial frame. If the current frame does not have a parent it is
      assumed to be an inertial frame.
      Note that the spatial acceleration is not the classical acceleration
      of the moving and accelerating body (@see getClassicAccel).
   */
  Vector6 getSpAccel(void) const
  {
    OpenFDMAssert(!mDisableSpAccel);
    return getRelAccel() + getParentSpAccel() + getHdot();
  }

  /** Classical acceleration of the current frame.
      @return The sensed acceleration of the current frame with respect to an
      inertial frame. If the current frame does not have a parent it is
      assumed to be an inertial frame.
   */
  Vector6 getClassicAccel(void) const
  {
    OpenFDMAssert(!mDisableSpAccel);
    Vector6 iv = getSpVel();
    return getRelAccel() + getParentSpAccel() + getHdot()
      + Vector6(Vector3::zeros(), cross(iv.getAngular(), iv.getLinear()));
  }


  /** FIXME
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

  /** Spatial inertia transform.
      Transforms a spatial inertia matrix from the current frame to the parent
      frame.
      @param I The inertia matrix in the current frame to be transformed.
      @return  The inertia matrix transformed to the parent frame.
   */
  SpatialInertia inertiaToParent(const SpatialInertia& I) const;

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
  { return motionTo(getRefPosition(), getRefOrientation(), v) - getRefVel(); }

  /** Spatial motion vector transform.
      Transforms a spatial motion vector from the current frame to the parent
      frame.
      @param v The motion vector in the current frame to be transformed.
      @return  The motion vector transformed to the parent frame.
   */
  Vector6 motionToRef(const Vector6& v) const
  { return motionFrom(getRefPosition(), getRefOrientation(), v + getRefVel()); }

  Plane planeFromRef(const Plane& plane) const
  { return planeTo(getRefPosition(), getRefOrientation(), plane); }

  Plane planeToRef(const Plane& plane) const
  { return planeFrom(getRefPosition(), getRefOrientation(), plane); }

  /** Reference orientation.
   * Returns the reference orientation of this frame wrt the topmost frame 
   * this frame is attached to.
   * It is measured in the topmost frames coordinates.
   */
  const Rotation& getRefOrientation(void) const
  {
    if (mDirtyPos)
      computePositionDep();
    return mRefOrient;
  }

  /** Reference position.
   * Returns the reference position of this frame wrt the topmost frame 
   * this frame is attached to.
   * It is measured in the topmost frames coordinates.
   */
  const Vector3& getRefPosition(void) const
  {
    if (mDirtyPos)
      computePositionDep();
    return mRefPos;
  }

protected:
  void setRefOrientation(const Quaternion& o)
  {
    if (hasParent())
      setOrientation(inverse(getParentFrame()->getRefOrientation())*o);
    else
      setOrientation(o);
  }

  void setRefPosition(const Vector3& p)
  {
    if (hasParent())
      setPosition(getParentFrame()->posFromRef(p));
    else
      setPosition(p);
  }

public:
  Quaternion getRelOrientation(const Frame* frame) const
  {
    OpenFDMAssert(frame->getRefFrameId() == getRefFrameId());
    return conjugate(getRefOrientation())*frame->getRefOrientation();
  }
  Vector3 getRelPosition(const Frame* frame) const
  {
    OpenFDMAssert(frame->getRefFrameId() == getRefFrameId());
    return getRelOrientation(frame).backTransform(frame->getRefPosition());
  }
  Vector6 getRelVel(const Frame* frame) const
  {
    OpenFDMAssert(frame->getRefFrameId() == getRefFrameId());
    return motionFromRef(frame->motionToRef(frame->getRefVel()));
  }

protected:
  void setPosition(const Vector3& p)
  { setPosDirty(); mPosition = p; }
  void setOrientation(const Quaternion& o)
  { setPosDirty(); mOrientation = o; }
  void setRelVel(const Vector6& vel)
  { setVelDirty(); mRelVel = vel; }
  void setLinearRelVel(const Vector3& v)
  { setVelDirty(); mRelVel.setLinear(v); }
  void setAngularRelVel(const Vector3& rotVel)
  { setVelDirty(); mRelVel.setAngular(rotVel); }
  void setRelAccel(const Vector6& accel)
  { setAccelDirty(); mRelAccel = accel; }
  void setLinearRelAccel(const Vector3& accel)
  { setAccelDirty(); mRelAccel.setLinear(accel); }
  void setAngularRelAccel(const Vector3& accel)
  { setAccelDirty(); mRelAccel.setAngular(accel); }

  void disableAccel(void)
  { mDisableSpAccel = true; }
  void enableAccel(void)
  { mDisableSpAccel = false; }

  void computePositionDep(void) const;
  void computeVelocityDep(void) const;
  void computeAccelerationDep(void) const;

protected:
  void setPosDirty(void)
  {
    // Don't bother iterating over all children if we are already dirty.
    if (mDirtyPos == true && mDirtySpVel == true && mDirtySpAccel == true)
      return;
    // Really set ourself and all children dirty.
    // Is done in this way to help the compiler inline the fast path and
    // only really call a function if real work needs to be done.
    setPosDirtyUnconditional();
  }
  void setVelDirty(void)
  {
    // Don't bother iterating over all children if we are already dirty.
    if (mDirtySpVel == true && mDirtySpAccel == true)
      return;
    // Really set ourself and all children dirty.
    // Is done in this way to help the compiler inline the fast path and
    // only really call a function if real work needs to be done.
    setVelDirtyUnconditional();
  }
  void setAccelDirty(void)
  {
    // Don't bother iterating over all children if we are already dirty.
    if (mDirtySpAccel == true)
      return;
    // Really set ourself and all children dirty.
    // Is done in this way to help the compiler inline the fast path and
    // only really call a function if real work needs to be done.
    setAccelDirtyUnconditional();
  }
private:
  void setPosDirtyUnconditional(void);
  void setVelDirtyUnconditional(void);
  void setAccelDirtyUnconditional(void);

private:
  /// Set the parent frame to the given one.
  void setParentFrame(Frame* parent)
  { mParentFrame = parent; }

  // The offset of this frames origin wrt the parent frame measured in
  // the parent frames coordinates.
  Vector3 mPosition;
  // The orientation wrt the parent frame (measured in the parent frames
  Rotation mOrientation;

  // The spatial velocity wrt parent frame measured in
  // this frames coordinates.
  // True? more the relative velocity ...
  Vector6 mRelVel;

  // The spatial acceleration of this frame wrt the parent frame.
  // True? more the relative acceleration ...
  Vector6 mRelAccel;

  mutable Vector6 mParentSpVel;
  mutable Vector6 mParentSpAccel;

  mutable Rotation mRefOrient;
  mutable Vector3 mRefPos;
  mutable Vector6 mRefVel;

  // Here the topmost frame's id is stored.
  // Is used to check for compatibility of reference values when frame relative
  // values are computed.
  mutable frameid_type mReferenceFrameId;

  // Flag which tells the frame if dependent values must
  // be recomputed or not.
  mutable bool mDirtyPos:1;
  mutable bool mDirtySpVel:1;
  mutable bool mDirtySpAccel:1;
  mutable bool mDisableSpAccel:1;

  // The parent frame.
  // FIXME: May be we should store a list of all parents ???
  managed_ptr<Frame> mParentFrame;
  // The list of child frames.
  ChildFrameList mChildFrames;

  /// The frame's name. Might move into Object
  std::string mName;
};

class FreeFrame
  : public Frame {
public:
  FreeFrame(const std::string& name = std::string())
    : Frame(name)
  {}
  virtual ~FreeFrame(void)
  {}

//   using Frame::setPosition;
  void setPosition(const Vector3& p)
  { Frame::setPosition(p); }
  void setOrientation(const Quaternion& o)
  { Frame::setOrientation(o); }
  void setRelVel(const Vector6& vel)
  { Frame::setRelVel(vel); }
  void setLinearRelVel(const Vector3& vel)
  { Frame::setLinearRelVel(vel); }
  void setAngularRelVel(const Vector3& vel)
  { Frame::setAngularRelVel(vel); }
  void setRelAccel(const Vector6& accel)
  { Frame::setRelAccel(accel); }
  void setLinearRelAccel(const Vector3& accel)
  { Frame::setLinearRelAccel(accel); }
  void setAngularRelAccel(const Vector3& accel)
  { Frame::setAngularRelAccel(accel); }

  void setRefPosition(const Vector3& p)
  { Frame::setRefPosition(p); }
  void setRefOrientation(const Quaternion& o)
  { Frame::setRefOrientation(o); }
};

} // namespace OpenFDM

#endif
