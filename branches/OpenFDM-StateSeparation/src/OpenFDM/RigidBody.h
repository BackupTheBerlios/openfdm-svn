/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RigidBody_H
#define OpenFDM_RigidBody_H

#include <string>
#include "MechanicInteractPort.h"
#include "MechanicNode.h"

namespace OpenFDM {

class RigidBody : public MechanicNode {
  OPENFDM_OBJECT(RigidBody, MechanicNode);
public:
  RigidBody(const std::string& name);
  virtual ~RigidBody();

  virtual void accept(NodeVisitor& visitor);

protected:
  MechanicInteractPort newMechanicInteractPort(const std::string& name)
  { return MechanicInteractPort(this, name); }
};

} // namespace OpenFDM

#endif
