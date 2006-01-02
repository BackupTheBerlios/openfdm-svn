/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include <string>
#include <vector>
#include <algorithm>

#include "LogStream.h"
#include "Object.h"
#include "Property.h"
#include "Variant.h"
#include "Port.h"

namespace OpenFDM {

Port::~Port(void)
{
}

void
Port::setPortInterface(PortInterface* portInterface)
{
  mPortInterface = portInterface;
  std::vector<SharedPtr<Port> >::iterator it;
  for (it = mChainPorts.begin(); it != mChainPorts.end(); ++it) {
    (*it)->setPortInterface(mPortInterface);
  }
}

bool
Port::isConnectedTo(const Port* sourcePort) const
{
  const Port* port = mSourcePort;
  while (port) {
    if (sourcePort == port)
      return true;
    port = port->mSourcePort;
  }
  
  return false;
}

bool
Port::hasSameSource(const Port* otherPort) const
{
  OpenFDMAssert(mPortInterface);
  return otherPort && otherPort->mPortInterface == mPortInterface;
}

Variant
Port::getValue(void)
{
  if (!mPortInterface)
    return Variant();

  RealPortInterface* realPortInterface
    = mPortInterface->toRealPortInterface();
  if (realPortInterface)
    return Variant(realPortInterface->getRealValue());

  MatrixPortInterface* matrixPortInterface
    = mPortInterface->toMatrixPortInterface();
  if (matrixPortInterface)
    return Variant(matrixPortInterface->getMatrixValue());

  return Variant();
}

void
Port::connect(Port* sourcePort)
{
  if (!sourcePort) {
    Log(Model, Warning) << "Part argument to Port::connect is zero!"
      "Ignoring!" << endl;
    return;
  }
  // disconnect from other source if we are already connected
  // FIXME: should this be explicit ??
  if (mSourcePort) {
    Log(Model, Warning) << "Connecting already connected port!"
      "Disconnecting from old port now." << endl;
    disconnect(mSourcePort);
  }
  
  // If we have a source port, propagate its context.
  setPortInterface(sourcePort->mPortInterface);
  sourcePort->mChainPorts.push_back(this);
  mSourcePort = sourcePort;
}

/// Disconnect this port from the given source port
void
Port::disconnect(Port* sourcePort)
{
  if (sourcePort != mSourcePort) {
    Log(Model, Error) << "Try to disconnect from source port we are not "
      "currently connected to!" << endl;
    return;
  }
  
  if (!mSourcePort)
    return;
  
  // Remove ourselves from the consumer list of the sourcePort to
  // disconnect us from
  std::vector<SharedPtr<Port> >::iterator it, beginPort, endPort;
  beginPort = sourcePort->mChainPorts.begin();
  endPort = sourcePort->mChainPorts.end();
  it = std::find(beginPort, endPort, this);
  if (it != endPort)
    sourcePort->mChainPorts.erase(it);
  
  // Reset our source
  mSourcePort = 0;
  
  // Invalidate all our listeners
  setPortInterface(0);
}

/// Just disconnect from whoever we are connected to
void
Port::disconnect(void)
{
  disconnect(mSourcePort);
}

} // namespace OpenFDM
