/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "Joint.h"

#include "ConstNodeVisitor.h"
#include "JointContext.h"
#include "LogStream.h"
#include "PortValueList.h"
#include "NodeVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Joint, MechanicNode)
  DEF_OPENFDM_PROPERTY(Vector3, Position, Serialized)
  END_OPENFDM_OBJECT_DEF

Joint::Joint(const std::string& name) :
  MechanicNode(name),
  mPosition(0, 0, 0)
{
}

Joint::~Joint(void)
{
}

void
Joint::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
Joint::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

MechanicContext*
Joint::newMechanicContext(const Environment* environment,
                          const MechanicLink* parentLink,
                          const MechanicLink* childLink,
                          PortValueList& portValueList) const
{
  MechanicLinkValue* parentLinkValue = 0;
  if (parentLink)
    parentLinkValue = portValueList.getPortValue(*parentLink);
  MechanicLinkValue* childLinkValue = 0;
  if (childLink)
    childLinkValue = portValueList.getPortValue(*childLink);
  return newJointContext(environment, parentLinkValue,
                         childLinkValue, portValueList);
}

const Vector3&
Joint::getPosition() const
{
  return mPosition;
}

void
Joint::setPosition(const Vector3& position)
{
  mPosition = position;
}

} // namespace OpenFDM
