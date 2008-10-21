/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "RootJoint.h"

#include "ConstNodeVisitor.h"
#include "NodeVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(RootJoint, Interact)
  DEF_OPENFDM_PROPERTY(Matrix, AngularBaseVelocity, Serialized)
  END_OPENFDM_OBJECT_DEF

RootJoint::RootJoint(const std::string& name) :
  Interact(name),
  mAngularBaseVelocity(Vector3::zeros())
{
}

RootJoint::~RootJoint()
{
}

void
RootJoint::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
RootJoint::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

} // namespace OpenFDM
