/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "RigidBody.h"

#include "ConstNodeVisitor.h"
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

PortId
RigidBody::addLink(const std::string& name)
{
  MechanicLink mechanicLink = newMechanicLink(name);
  mMechanicLinks.push_back(mechanicLink);
  return getPortId(mechanicLink.getPortIndex());
}

void
RigidBody::removeLink(const PortId& portId)
{
  MechanicLinkVector::iterator i = mMechanicLinks.begin();
  while (i != mMechanicLinks.end()) {
    if (getPortIndex(portId) == i->getPortIndex()) {
      i->clear();
      i = mMechanicLinks.erase(i);
    } else
      ++i;
  }
}

void
RigidBody::velocity(const Task&, const ContinousStateValueVector&,
                    PortValueList& portValues, FrameData&) const
{
  unsigned numLinkValues = mMechanicLinks.size();

  const Frame& frame = portValues[mMechanicLinks.front()].getFrame();
  for (unsigned i = 1; i < numLinkValues; ++i)
    portValues[mMechanicLinks[i]].getFrame().setPosAndVel(frame);
}

void
RigidBody::articulation(const Task&, const ContinousStateValueVector&,
                        PortValueList& portValues, FrameData&) const
{
  MechanicLinkValue& parentLink = portValues[mMechanicLinks.front()];

  unsigned numLinkValues = mMechanicLinks.size();
  for (unsigned i = 1; i < numLinkValues; ++i)
    parentLink.applyArticulation(portValues[mMechanicLinks[i]]);
}

void
RigidBody::acceleration(const Task&, const ContinousStateValueVector&,
                        PortValueList& portValues, FrameData&) const
{
  unsigned numLinkValues = mMechanicLinks.size();
  const Frame& frame = portValues[mMechanicLinks.front()].getFrame();
  for (unsigned i = 1; i < numLinkValues; ++i)
    portValues[mMechanicLinks[i]].getFrame().setAccel(frame);
}

} // namespace OpenFDM
