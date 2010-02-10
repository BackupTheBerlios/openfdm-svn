/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#include "NodeInstance.h"

#include "Assert.h"

namespace OpenFDM {

NodeInstance::NodeInstance(const SampleTime& sampleTime, const Node* node,
                           const PortValueList& portValueList) :
  AbstractNodeInstance(sampleTime),
  mNode(node),
  mPortValueList(portValueList)
{
  for (unsigned i = 0; i < mNode->getNumPorts(); ++i)
    OpenFDMAssert(mPortValueList.getPortValue(mNode->getPort(i)));
}

NodeInstance::~NodeInstance()
{
}

const Node&
NodeInstance::getNode() const
{
  return *mNode;
}

const PortValue*
NodeInstance::getPortValue(const Port& portInfo) const
{
  return mPortValueList.getPortValue(portInfo);
}

const NumericPortValue*
NodeInstance::getPortValue(const NumericPort& portInfo) const
{
  return mPortValueList.getPortValue(portInfo);
}

const MechanicLinkValue*
NodeInstance::getPortValue(const MechanicLink& portInfo) const
{
  return mPortValueList.getPortValue(portInfo);
}

} // namespace OpenFDM
