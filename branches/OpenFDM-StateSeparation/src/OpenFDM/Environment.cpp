/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Environment.h"

namespace OpenFDM {

Environment::Environment() :
  mInertial(new AbstractInertial),
  mGravity(new AbstractGravity),
  mWind(new AbstractWind)
{
}

Environment::~Environment(void)
{
}

} // namespace OpenFDM
