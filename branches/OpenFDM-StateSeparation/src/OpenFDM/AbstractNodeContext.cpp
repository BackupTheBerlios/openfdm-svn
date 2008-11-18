/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "AbstractNodeContext.h"

namespace OpenFDM {

AbstractNodeContext::AbstractNodeContext()
{
}

AbstractNodeContext::~AbstractNodeContext()
{
}

const PortValue*
AbstractNodeContext::getPortValue(const PortInfo& portInfo) const
{
  return mPortValueList.getPortValue(portInfo);
}

const NumericPortValue*
AbstractNodeContext::getPortValue(const NumericPortInfo& portInfo) const
{
  return mPortValueList.getPortValue(portInfo);
}

const MechanicLinkValue*
AbstractNodeContext::getPortValue(const MechanicLinkInfo& portInfo) const
{
  return mPortValueList.getPortValue(portInfo);
}

void
AbstractNodeContext::setPortValue(const PortInfo& portInfo,
                                  PortValue* portValue)
{
  mPortValueList.setPortValue(portInfo.getIndex(), portValue);
}

} // namespace OpenFDM
