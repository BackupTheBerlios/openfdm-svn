/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Vector.h"
#include "ConstantForce.h"

namespace OpenFDM {

ConstantForce::ConstantForce(const std::string& name, const Vector6& force)
  : ExternalForce(name), mForce(force)
{
}

ConstantForce::~ConstantForce(void)
{
}

void
ConstantForce::computeForce(void)
{
  // FIXME: do once ...
  applyForce(forceFrom(mPosition, mOrientation, mForce));
}

} // namespace OpenFDM
