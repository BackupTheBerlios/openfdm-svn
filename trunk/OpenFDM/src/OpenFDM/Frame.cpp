/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Frame.h"

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "FrameVisitor.h"
#include "ConstFrameVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Frame, Object)
  DEF_OPENFDM_ROPROP(Vector3, Position)
  DEF_OPENFDM_ROPROP(Vector3, RefPosition)
  DEF_OPENFDM_ROPROP(Quaternion, Orientation)
  DEF_OPENFDM_ROPROP(Quaternion, RefOrientation)
  DEF_OPENFDM_ROPROP(Vector6, RelVel)
  DEF_OPENFDM_ROPROP(Vector6, SpVel)
  DEF_OPENFDM_ROPROP(Vector6, RefVel)
  DEF_OPENFDM_ROPROP(Vector6, RelVelDot)
  DEF_OPENFDM_ROPROP(Vector6, SpAccel)
  END_OPENFDM_OBJECT_DEF

/// FIXME
BEGIN_OPENFDM_OBJECT_DEF(FreeFrame, Frame)
  END_OPENFDM_OBJECT_DEF

Frame::Frame(const std::string& name) :
  Object(name),
  mDirtyPos(true),
  mDirtySpVel(true),
  mDirtySpAccel(true),
  mReferenceFrameId(0)
{
  setPosition(Vector3::zeros());
  setOrientation(Quaternion::unit());
  setRelVel(Vector6::zeros());
}

Frame::~Frame(void)
{
}

void
Frame::accept(FrameVisitor& visitor)
{
  visitor.apply(*this);
}

void
Frame::traverse(FrameVisitor& visitor)
{
  ChildFrameList::iterator it = mChildFrames.begin();
  ChildFrameList::iterator iEnd = mChildFrames.end();
  while (it != iEnd) {
    (*it)->accept(visitor);
    ++it;
  }
}

void
Frame::accept(ConstFrameVisitor& visitor) const
{
  visitor.apply(*this);
}

void
Frame::traverse(ConstFrameVisitor& visitor) const
{
  ChildFrameList::const_iterator it = mChildFrames.begin();
  ChildFrameList::const_iterator iEnd = mChildFrames.end();
  while (it != iEnd) {
    (*it)->accept(visitor);
    ++it;
  }
}

bool
Frame::isParentFrameOf(const Frame* const frame) const
{
  return isDirectParentFrameOf(frame) ||
    (frame && isParentFrameOf(frame->mParentFrame.lock()));
}

bool
Frame::isChildFrameOf(const Frame* const frame) const
{
  SharedPtr<Frame> parent = mParentFrame.lock();
  return isDirectChildFrameOf(frame) ||
    (parent && parent->isChildFrameOf(frame));
}

bool
Frame::addChildFrame(Frame* child)
{
  if (!child) {
    Log(Frame,Warning) << "Trying to attach zero pointer child Frame to "
                       << "Frame \"" << getName() << "\"!" << endl;
    return false;
  }
  if (child->getParentFrame()) {
    Log(Frame,Error) << "Can not attach Frame \"" << child->getName()
                     << "\" to Frame \"" << getName() << "\": "
                     << " is already child of \""
                     << child->getParentFrame()->getName() << endl;
    return false;
  }
  
  child->setParentFrame(this);
  mChildFrames.push_back(child);
  return true;
}

bool
Frame::removeChildFrame(Frame* child)
{
  ChildFrameList::iterator it = mChildFrames.begin();
  while (it != mChildFrames.end()) {
    if ((*it) == child) {
      it = mChildFrames.erase(it);
      return true;
    }
    ++it;
  }

  return false;
}

Frame*
Frame::getChildFrame(unsigned i)
{
  if (mChildFrames.size() <= i)
    return 0;
  return mChildFrames[i];
}

const Frame*
Frame::getChildFrame(unsigned i) const
{
  if (mChildFrames.size() <= i)
    return 0;
  return mChildFrames[i];
}

unsigned
Frame::getChildFrameIndex(const Frame* child) const
{
  if (!child)
    return mChildFrames.size();

  unsigned i = 0;
  for (; i < mChildFrames.size(); ++i) {
    if (mChildFrames[i] == child)
      return i;
  }
  return i;
}

void
Frame::reparentChildren(Frame* frame)
{
  if (!frame)
    return;

  ChildFrameList::iterator it = frame->mChildFrames.begin();
  while (it != frame->mChildFrames.end()) {
    Log(Model,Error) << "Moving Frame " << (*it)->getName() << " from "
                     << frame->getName() << " to " << getName() << endl;
    (*it)->setParentFrame(this);
    mChildFrames.push_back(*it);
    it = frame->mChildFrames.erase(it);
  }
}

void
Frame::computePositionDep(void) const
{
  if (hasParent()) {
    mRefOrient = getParentFrame()->getRefOrientation()*getOrientation();
    mRefPos = getParentFrame()->posToRef(getPosition());
    mReferenceFrameId = getParentFrame()->getRefFrameId();
  } else {
    mRefOrient = getOrientation();
    mRefPos = getPosition();
    mReferenceFrameId = getFrameId();
  }
  mDirtyPos = false;
}

void
Frame::computeVelocityDep(void) const
{
  if (hasParent()) {
    mParentSpVel = motionFromParent(getParentFrame()->getSpVel());
    mRefVel = getRelVel() + motionFromParent(getParentFrame()->getRefVel());
    mReferenceFrameId = getParentFrame()->getRefFrameId();
  } else {
    mParentSpVel = Vector6::zeros();
    mRefVel = Vector6::zeros();
    mReferenceFrameId = getFrameId();
  }
  mDirtySpVel = false;
}

void
Frame::computeAccelerationDep(void) const
{
  if (hasParent()) {
    mParentSpAccel = motionFromParent(getParentFrame()->getSpAccel());
    mReferenceFrameId = getParentFrame()->getRefFrameId();
  } else {
    mParentSpAccel = Vector6::zeros();
    mReferenceFrameId = getFrameId();
  }
  mDirtySpAccel = false;
}

void
Frame::setPosDirtyUnconditional(void)
{
  // Mark ourself dirty.
  mDirtyPos = true;
  mDirtySpVel = true;
  mDirtySpAccel = true;
  
  // Mark all child dirty.
  ChildFrameList::iterator it = mChildFrames.begin();
  ChildFrameList::iterator iEnd = mChildFrames.end();
  while (it != iEnd) {
    (*it)->setPosDirty();
    ++it;
  }
}

void
Frame::setVelDirtyUnconditional(void)
{
  // Mark ourself dirty.
  mDirtySpVel = true;
  mDirtySpAccel = true;
  
  // Mark all child dirty.
  ChildFrameList::iterator it = mChildFrames.begin();
  ChildFrameList::iterator iEnd = mChildFrames.end();
  while (it != iEnd) {
    (*it)->setVelDirty();
    ++it;
  }
}

void
Frame::setAccelDirtyUnconditional(void) const
{
  // Mark ourself dirty.
  mDirtySpAccel = true;
  
  // Mark all child dirty.
  ChildFrameList::const_iterator it = mChildFrames.begin();
  ChildFrameList::const_iterator iEnd = mChildFrames.end();
  while (it != iEnd) {
    (*it)->setAccelDirty();
    ++it;
  }
}

void
Frame::setParentFrame(Frame* parent)
{
  mParentFrame = parent;
  setPosDirty();
}

SpatialInertia
Frame::inertiaToParent(const SpatialInertia& I) const
{
  if (getOrientation().isIdentity()) {
    if (getPosition() == Vector3::zeros()) {
      return I;
    } else {
      return inertiaFrom(getPosition(), I);
    }
  } else {
    if (getPosition() == Vector3::zeros()) {
      return inertiaFrom(getOrientation(), I);
    } else {
      return inertiaFrom(getPosition(), getOrientation(), I);
    }
  }
}

const Vector6 Frame::mZeroVector = Vector6::zeros();

} // namespace OpenFDM
