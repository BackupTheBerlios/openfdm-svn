/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "MechanicContext.h"

namespace OpenFDM {

MechanicContext::MechanicContext(const Environment* environment) :
  mEnvironment(environment)
{
  OpenFDMAssert(mEnvironment);
}

MechanicContext::~MechanicContext()
{
}

} // namespace OpenFDM
