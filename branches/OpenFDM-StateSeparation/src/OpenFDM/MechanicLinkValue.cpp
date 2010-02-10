/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#include "MechanicLinkValue.h"

namespace OpenFDM {

MechanicLinkValue::MechanicLinkValue() :
  mVelocity(Vector6::zeros()),
  mInertialVelocity(Vector6::zeros()),
  mInertialAcceleration(Vector6::zeros()),
  mForce(Vector6::zeros()),
  mInertia(SpatialInertia::zeros())
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
