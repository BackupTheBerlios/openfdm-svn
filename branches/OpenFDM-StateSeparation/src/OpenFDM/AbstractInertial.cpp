/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "AbstractInertial.h"

namespace OpenFDM {

AbstractInertial::~AbstractInertial()
{
}

Vector3
AbstractInertial::getAngularVelocity(const real_type& t) const
{
  // return Vector3(0, 0, pi2/(24*60*60));
  return Vector3::zeros();
}

Vector6
AbstractInertial::getAcceleration(const real_type& t) const
{
  return Vector6::zeros();
}

} // namespace OpenFDM
