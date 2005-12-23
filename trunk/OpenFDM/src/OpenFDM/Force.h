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

  /**
   */
  virtual const Vector6& getForce(Frame *parent) const = 0;

  // Needs to call applyForce once ...
  virtual void computeForce(void) = 0;
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
