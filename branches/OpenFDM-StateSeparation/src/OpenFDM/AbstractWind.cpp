/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "AbstractWind.h"

namespace OpenFDM {

AbstractWind::~AbstractWind()
{
}

Vector6
AbstractWind::getWindVelocity(const Environment&, const real_type& t,
                              const Vector3&) const
{
  return Vector6::zeros();
}

} // namespace OpenFDM
