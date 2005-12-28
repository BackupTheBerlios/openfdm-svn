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
#include "Interact.h"

namespace OpenFDM {

class Force :
    public Interact {
public:
  Force(const std::string& name, unsigned numParents);
  virtual ~Force(void);

};

class ExternalForce
  : public Force {
public:
  ExternalForce(const std::string& name)
    : Force(name, 1), mForce(0, 0, 0, 0, 0, 0)
  {}
  virtual ~ExternalForce(void) {}

  // Needs to call applyForce once ...
  virtual void interactWith(RigidBody* rigidBody)
  {
    computeForce();
    rigidBody->contributeForce(-mForce);
  }

  /// FIXME here for compatibility
  virtual void computeForce(void) = 0;

protected:
  /** Sets the force contribution of this force element.
   * Sets the force contribution of this current force element to
   * the parent rigid body to force.
   */
  void applyForce(const Vector6& force)
  {
    mForce = force;
  }

  /// FIXME remove
  const Frame* getParentFrame(unsigned id = 0) const
  {
    OpenFDMAssert(id < mParents.size() && mParents[id]);
    return getParentRigidBody(id)->getFrame();
  }
  /// FIXME remove
  Frame* getParentFrame(unsigned id = 0)
  {
    OpenFDMAssert(id < mParents.size() && mParents[id]);
    return getParentRigidBody(id)->getFrame();
  }

private:
  Vector6 mForce;
};

} // namespace OpenFDM

#endif
