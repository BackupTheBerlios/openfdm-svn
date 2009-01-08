/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#include "AbstractNodeInstance.h"

#include "Assert.h"
#include "Node.h"
#include "SharedPtr.h"
#include "WeakReferenced.h"

namespace OpenFDM {

AbstractNodeInstance::AbstractNodeInstance(const SampleTime& sampleTime) :
  mSampleTime(sampleTime)
{
}

AbstractNodeInstance::~AbstractNodeInstance()
{
}

const NumericPortValue*
AbstractNodeInstance::getPortValue(const NumericPortInfo& portInfo) const
{
  const PortValue* portValue;
  portValue = getPortValue(static_cast<const PortInfo&>(portInfo));
  if (!portValue)
    return 0;
  return portValue->toNumericPortValue();
}

const MechanicLinkValue*
AbstractNodeInstance::getPortValue(const MechanicLinkInfo& portInfo) const
{
  const PortValue* portValue;
  portValue = getPortValue(static_cast<const PortInfo&>(portInfo));
  if (!portValue)
    return 0;
  return portValue->toMechanicLinkValue();
}

} // namespace OpenFDM
