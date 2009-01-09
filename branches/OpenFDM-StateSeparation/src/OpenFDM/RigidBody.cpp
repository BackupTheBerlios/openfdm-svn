/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#include "RigidBody.h"

#include "ConstNodeVisitor.h"
#include "MechanicContext.h"
#include "NodeVisitor.h"
#include "PortValueList.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(RigidBody, MechanicNode)
  END_OPENFDM_OBJECT_DEF

RigidBody::RigidBody(const std::string& name) :
  MechanicNode(name)
{
  addLink("link0");
  addLink("link1");
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

const MechanicLinkInfo*
RigidBody::addLink(const std::string& name)
{
  MechanicLinkInfo* mechanicLinkInfo = new MechanicLinkInfo(this, name);
  mMechanicLinks.push_back(mechanicLinkInfo);
  return mechanicLinkInfo;
}

void
RigidBody::removeLink(const MechanicLinkInfo* mechanicLinkInfo)
{
  MechanicLinkInfoVector::iterator i = mMechanicLinks.begin();
  while (i != mMechanicLinks.end()) {
    if (i->get() == mechanicLinkInfo) {
      i->clear();
      i = mMechanicLinks.erase(i);
    } else
      ++i;
  }
}

} // namespace OpenFDM
