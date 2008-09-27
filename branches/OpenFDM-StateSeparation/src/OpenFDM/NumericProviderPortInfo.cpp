/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "NumericProviderPortInfo.h"

#include "NumericPortValue.h"

namespace OpenFDM {

NumericProviderPortInfo::NumericProviderPortInfo(Node* node, const std::string& name, const Size& size) :
  ProviderPortInfo(node, name),
  mSize(size)
{
}

NumericProviderPortInfo::~NumericProviderPortInfo()
{
}

NumericPortValue*
NumericProviderPortInfo::newValueImplementation() const
{
  return new NumericPortValue(mSize);
}

} // namespace OpenFDM
