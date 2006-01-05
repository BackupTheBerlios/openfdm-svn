/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Vector.h"
#include "ConstantForce.h"

namespace OpenFDM {

ConstantForce::ConstantForce(const std::string& name, const Vector6& force)
  : ExternalForce(name)
{
  setForce(force);
}

ConstantForce::~ConstantForce(void)
{
}

} // namespace OpenFDM
