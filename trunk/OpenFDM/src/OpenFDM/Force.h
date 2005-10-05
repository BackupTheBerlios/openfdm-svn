/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Force_H
#define OpenFDM_Force_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "RigidBody.h"
#include "Visitor.h"
#include "ConstVisitor.h"

namespace OpenFDM {

class Force
  : public MultiBodyModel {
public:
  Force(const std::string& name);
  virtual ~Force(void);

  virtual void accept(Visitor& visitor);
  virtual void accept(ConstVisitor& visitor) const;

  virtual Force* toForce(void);
  virtual const Force* toForce(void) const;

  void applyForce(RigidBody& body) const
  {
    // What about that??
    if (&body == getParentFrame(0)) {
    } else if (&body == getParentFrame(1)) {
    } else {
      // error
    }
  }

  /**
   */
  virtual const Vector6& getForce(Frame *parent) const = 0;

  // Needs to call applyForce once ...
  virtual void computeForce(void) = 0;
};

class InternalForce
  : public Force {
public:
  OpenFDM_NodeImplementation(2);

  InternalForce(const std::string& name)
    : Force(name)
  {}
  virtual ~InternalForce(void) {}

  virtual const Vector6& getForce(Frame *parent) const
  {
    OpenFDMAssert(parent == getParentFrame(0) || parent == getParentFrame(1));
    if (getParentFrame(0) == parent)
      return mForce[0];
    else
      return mForce[1];
  }

protected:
  /**
   */
 
  Vector6 parentXTransform(unsigned fromParent, unsigned toParent,
                           const Vector6& force)
  {
    Frame* parentFrames[2] = { getParentFrame(fromParent), getParentFrame(toParent) };
    if (!(parentFrames[0] && parentFrames[1]))
      return Vector6();

    // FIXME don't go over the world's center ...
    Vector6 refForce = forceFrom(parentFrames[0]->getRefPosition(),
                                 parentFrames[0]->getRefOrientation(), force);

    return forceTo(parentFrames[1]->getRefPosition(),
                   parentFrames[1]->getRefOrientation(), refForce);
  }

  /** Sets the force contribution of this force element.
   * Sets the force contribution of this current force element to
   * the parent rigid body with the index parent to force.
   * The force applied to the other rigid body is transformed accordingly.
   */
  void applyForce(unsigned parent, const Vector6& force)
  {
    OpenFDMAssert(parent < 2);
    if (2 <= parent)
      return;

    if (parent == 0) {
      mForce[0] = force;
      mForce[1] = parentXTransform(0, 1, force);
    } else {
      mForce[0] = parentXTransform(1, 0, force);
      mForce[1] = force;
    }
  }

private:
  Vector6 mForce[2];
};

class ExternalForce
  : public Force {
  OpenFDM_NodeImplementation(1);
public:
  ExternalForce(const std::string& name)
    : Force(name)
  {}
  virtual ~ExternalForce(void) {}

  virtual const Vector6& getForce(Frame *parent) const
  {
    OpenFDMAssert(parent == getParentFrame(0));
    return mForce;
  }

protected:
  /** Sets the force contribution of this force element.
   * Sets the force contribution of this current force element to
   * the parent rigid body to force.
   */
  void applyForce(const Vector6& force)
  {
    mForce = force;
  }

private:
  Vector6 mForce;
};

} // namespace OpenFDM

#endif
