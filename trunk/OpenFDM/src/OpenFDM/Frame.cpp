/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "FrameVisitor.h"
#include "ConstFrameVisitor.h"

namespace OpenFDM {

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

  addProperty("position", Property(this, &Frame::getPosition));
  addProperty("orienatation", Property(this, &Frame::getOrientation));
  addProperty("relVel", Property(this, &Frame::getRelVel));
  addProperty("spVel", Property(this, &Frame::getSpVel));
  addProperty("refVel", Property(this, &Frame::getRefVel));
  addProperty("relVelDot", Property(this, &Frame::getRelVelDot));
  addProperty("spAccel", Property(this, &Frame::getSpAccel));
  addProperty("classicAccel", Property(this, &Frame::getClassicAccel));
  addProperty("refPosition", Property(this, &Frame::getRefPosition));
  addProperty("refOrienatation", Property(this, &Frame::getRefOrientation));
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
    (frame && isParentFrameOf(frame->mParentFrame));
}

bool
Frame::isChildFrameOf(const Frame* const frame) const
{
  return isDirectChildFrameOf(frame) ||
    (mParentFrame && mParentFrame->isChildFrameOf(frame));
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
