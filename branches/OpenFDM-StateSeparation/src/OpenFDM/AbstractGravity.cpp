/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "AbstractGravity.h"

namespace OpenFDM {

AbstractGravity::~AbstractGravity()
{
}

Vector3
AbstractGravity::getGravityAcceleration(const Environment&,
                                        const Vector3&) const
{
  return Vector3(0, 0, 9.81);
}

} // namespace OpenFDM
