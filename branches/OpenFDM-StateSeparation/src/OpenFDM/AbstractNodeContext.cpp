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
  return mPortValueList.getPortValue(portInfo.getIndex());
}

} // namespace OpenFDM
