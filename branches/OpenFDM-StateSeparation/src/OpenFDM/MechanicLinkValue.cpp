/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#include "MechanicLinkValue.h"

namespace OpenFDM {

MechanicLinkValue::MechanicLinkValue() :
  mArticulatedForce(Vector6::zeros()),
  mArticulatedInertia(SpatialInertia::zeros())
{
}

MechanicLinkValue::~MechanicLinkValue()
{
}

MechanicLinkValue*
MechanicLinkValue::toMechanicLinkValue()
{
  return this;
}

const MechanicLinkValue*
MechanicLinkValue::toMechanicLinkValue() const
{
  return this;
}

} // namespace OpenFDM
