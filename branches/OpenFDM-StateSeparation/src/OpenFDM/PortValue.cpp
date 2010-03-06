/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich
 *
 */

#include "PortValue.h"

namespace OpenFDM {

PortValue::~PortValue()
{
}

void
PortValue::destruct(const PortValue* portValue)
{
  delete portValue;
}

NumericPortValue*
PortValue::toNumericPortValue()
{
  return 0;
}

const NumericPortValue*
PortValue::toNumericPortValue() const
{
  return 0;
}

MechanicLinkValue*
PortValue::toMechanicLinkValue()
{
  return 0;
}

const MechanicLinkValue*
PortValue::toMechanicLinkValue() const
{
  return 0;
}

} // namespace OpenFDM
