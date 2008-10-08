/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "ProviderPortInfo.h"

namespace OpenFDM {

ProviderPortInfo::ProviderPortInfo(Node* node, const std::string& name) :
  PortInfo(node, name)
{
}

ProviderPortInfo::~ProviderPortInfo()
{
}

const ProviderPortInfo*
ProviderPortInfo::toProviderPortInfo() const
{
  return this;
}

} // namespace OpenFDM
