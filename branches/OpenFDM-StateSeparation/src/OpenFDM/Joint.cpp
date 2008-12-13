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
  END_OPENFDM_OBJECT_DEF

Joint::Joint(const std::string& name) :
  MechanicNode(name)
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

} // namespace OpenFDM
