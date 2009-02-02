/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#include "RootJoint.h"

#include "ConstNodeVisitor.h"
#include "NodeVisitor.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(RootJoint, Joint)
  END_OPENFDM_OBJECT_DEF

RootJoint::RootJoint(const std::string& name) :
  Joint(name)
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
