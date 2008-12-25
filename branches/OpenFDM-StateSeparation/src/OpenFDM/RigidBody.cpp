/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
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

const PortInfo*
RigidBody::addLink(const std::string& name)
{
  // FIXME: simplify
  MechanicLink mechanicLink = newMechanicLink(name);
  mMechanicLinks.push_back(mechanicLink);
  return getPort(mechanicLink.getPortIndex());
}

void
RigidBody::removeLink(const PortInfo* portInfo)
{
  // FIXME: simplify
  MechanicLinkVector::iterator i = mMechanicLinks.begin();
  while (i != mMechanicLinks.end()) {
    if (getPort(i->getPortIndex()) == portInfo) {
      i->clear();
      i = mMechanicLinks.erase(i);
    } else
      ++i;
  }
}

} // namespace OpenFDM
