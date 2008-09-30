/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "MechanicContext.h"

namespace OpenFDM {

MechanicContext::MechanicContext(const MechanicNode* mechanicNode) :
  mMechanicNode(mechanicNode)
{
  OpenFDMAssert(mMechanicNode);
}

MechanicContext::~MechanicContext()
{
}

const MechanicNode&
MechanicContext::getNode() const
{
  return *mMechanicNode;
}

bool
MechanicContext::isConnectedTo(const MechanicContext& mechanicContext) const
{
  unsigned numPorts = mMechanicNode->getNumPorts();
  for (unsigned i = 0; i < numPorts; ++i) {
    SharedPtr<const PortInfo> portInfo = mMechanicNode->getPort(i);
    OpenFDMAssert(portInfo);
    const PortValue* portValue = getPortValueList().getPortValue(i);
    if (!portValue)
      continue;
    unsigned otherNumPorts = mechanicContext.mMechanicNode->getNumPorts();
    for (unsigned j = 0; j < otherNumPorts; ++j) {
      if (!mechanicContext.mMechanicNode->getPort(j)->toProviderPortInfo())
        continue;
      
      const PortValue* otherPortValue;
      otherPortValue = mechanicContext.getPortValueList().getPortValue(j);
      if (portValue != otherPortValue)
        continue;
      
      return true;
    }
  }
  return false;
}

} // namespace OpenFDM
