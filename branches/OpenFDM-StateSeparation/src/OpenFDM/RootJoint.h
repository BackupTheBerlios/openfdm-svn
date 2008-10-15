/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RootJoint_H
#define OpenFDM_RootJoint_H

#include <string>
#include "Interact.h"

namespace OpenFDM {

class RootJoint : public Interact {
  OPENFDM_OBJECT(RootJoint, Interact);
public:
  RootJoint(const std::string& name);
  virtual ~RootJoint();

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  const Vector3& getAngularBaseVelocity() const
  { return mAngularBaseVelocity; }
  void setAngularBaseVelocity(const Vector3& angularBaseVelocity)
  { mAngularBaseVelocity = angularBaseVelocity; }

private:
  Vector3 mAngularBaseVelocity;
};

} // namespace OpenFDM

#endif
