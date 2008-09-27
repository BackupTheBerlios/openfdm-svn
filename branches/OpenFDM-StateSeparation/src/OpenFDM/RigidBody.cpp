/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "RigidBody.h"

#include "ConstNodeVisitor.h"
#include "NodeVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(RigidBody, MechanicNode)
  END_OPENFDM_OBJECT_DEF

RigidBody::RigidBody(const std::string& name) :
  MechanicNode(name)
{
}

RigidBody::~RigidBody()
{
}

void
RigidBody::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
RigidBody::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

} // namespace OpenFDM
