/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
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

} // namespace OpenFDM
