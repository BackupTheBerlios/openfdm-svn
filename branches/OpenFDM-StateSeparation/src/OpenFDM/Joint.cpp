/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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
                          const MechanicLinkInfo* parentLinkInfo,
                          const MechanicLinkInfo* childLinkInfo,
                          PortValueList& portValueList) const
{
  MechanicLinkValue* parentLinkValue = 0;
  if (parentLinkInfo)
    parentLinkValue = portValueList.getPortValue(*parentLinkInfo);
  MechanicLinkValue* childLinkValue = 0;
  if (childLinkInfo)
    childLinkValue = portValueList.getPortValue(*childLinkInfo);
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
