/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "RootJoint.h"

#include "ConstNodeVisitor.h"
#include "NodeVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(RootJoint, Interact)
  END_OPENFDM_OBJECT_DEF

RootJoint::RootJoint(const std::string& name) :
  Interact(name)
{
}

RootJoint::~RootJoint()
{
}

void
RootJoint::accept(NodeVisitor& visitor)
{
  visitor.apply(*this);
}

void
RootJoint::accept(ConstNodeVisitor& visitor) const
{
  visitor.apply(*this);
}

} // namespace OpenFDM