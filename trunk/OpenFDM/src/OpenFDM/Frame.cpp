/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Inertia.h"
#include "Frame.h"
#include "Visitor.h"
#include "ConstVisitor.h"

namespace OpenFDM {

Frame::Frame(const std::string& name) :
  mName(name),
  mDirtyPos(true),
  mDirtySpVel(true),
  mDirtySpAccel(true),
  mDisableSpAccel(false),
  mReferenceFrameId(0)
{
  setPosition(Vector3::zeros());
  setOrientation(Quaternion::unit());
  setRelVel(Vector6::zeros());
  setRelAccel(Vector6::zeros());

  addProperty("name", Property(this, &Frame::getName, &Frame::setName));
  addProperty("position", Property(this, &Frame::getPosition));
  addProperty("orienatation", Property(this, &Frame::getOrientation));
  addProperty("relVel", Property(this, &Frame::getRelVel));
  addProperty("spVel", Property(this, &Frame::getSpVel));
  addProperty("refVel", Property(this, &Frame::getRefVel));
  addProperty("relAccel", Property(this, &Frame::getRelAccel));
  addProperty("spAccel", Property(this, &Frame::getSpAccel));
  addProperty("classicAccel", Property(this, &Frame::getClassicAccel));
  addProperty("refPosition", Property(this, &Frame::getRefPosition));
  addProperty("refOrienatation", Property(this, &Frame::getRefOrientation));
}

Frame::~Frame(void)
{
}

void
Frame::accept(Visitor& visitor)
{
  visitor.apply(*this);
}

void
Frame::traverse(Visitor& visitor)
{
  {
    ChildFrameList::iterator it = mChildFrames.begin();
    ChildFrameList::iterator iEnd = mChildFrames.end();
    while (it != iEnd) {
      (*it)->accept(visitor);
      ++it;
    }
  }
  abchild_iterator it = _children.begin();
  abchild_iterator iEnd = _children.end();
  while (it != iEnd) {
    // Make sure the nodes with two parents are only traversed once.
    MultiBodyModel* child = (*it);
    if (child->getParentFrame(0) == this)
      child->accept(visitor);
    
    ++it;
  }
}

void
Frame::accept(ConstVisitor& visitor) const
{
  visitor.apply(*this);
}

void
Frame::traverse(ConstVisitor& visitor) const
{
  {
    ChildFrameList::const_iterator it = mChildFrames.begin();
    ChildFrameList::const_iterator iEnd = mChildFrames.end();
    while (it != iEnd) {
      (*it)->accept(visitor);
      ++it;
    }
  }
  const_abchild_iterator it = _children.begin();
  const_abchild_iterator iEnd = _children.end();
  while (it != iEnd) {
    // Make sure the nodes with two parents are only traversed once.
    const MultiBodyModel* child = (*it);
    if (child->getParentFrame(0) == this)
      child->accept(visitor);
    
    ++it;
  }
}

RigidBody*
Frame::toRigidBody(void)
{
  return 0;
}

const RigidBody*
Frame::toRigidBody(void) const
{
  return 0;
}

bool
Frame::addChildFrame(Frame* child)
{
  if (!child)
    return false;
  // check if it is already there. Emit an error in this caes.
  ChildFrameList::iterator it = mChildFrames.begin();
  ChildFrameList::iterator iEnd = mChildFrames.end();
  while (it != iEnd) {
    if ((*it) == child)
      return false;
    ++it;
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
Frame::setAccelDirtyUnconditional(void)
{
  // Mark ourself dirty.
  mDirtySpAccel = true;
  
  // Mark all child dirty.
  ChildFrameList::iterator it = mChildFrames.begin();
  ChildFrameList::iterator iEnd = mChildFrames.end();
  while (it != iEnd) {
    (*it)->setAccelDirty();
    ++it;
  }
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

} // namespace OpenFDM
