/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "MechanicContext.h"

namespace OpenFDM {

MechanicContext::~MechanicContext()
{
}

bool
MechanicContext::isConnectedTo(const MechanicContext& mechanicContext) const
{
  unsigned numPorts = getNode().getNumPorts();
  for (unsigned i = 0; i < numPorts; ++i) {
    SharedPtr<const PortInfo> portInfo = getNode().getPort(i);
    OpenFDMAssert(portInfo);
    const PortValue* portValue = getPortValueList().getPortValue(i);
    if (!portValue)
      continue;
    unsigned otherNumPorts = mechanicContext.getNode().getNumPorts();
    for (unsigned j = 0; j < otherNumPorts; ++j) {
      if (!mechanicContext.getNode().getPort(j)->toMechanicLinkInfo())
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
