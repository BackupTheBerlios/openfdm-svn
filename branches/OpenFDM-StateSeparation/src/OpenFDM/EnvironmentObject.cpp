/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "EnvironmentObject.h"
#include "Object.h"
#include "Environment.h"

namespace OpenFDM {

EnvironmentObject::EnvironmentObject(void)
{
}

EnvironmentObject::~EnvironmentObject(void)
{
}

void
EnvironmentObject::attachToEnvironment(Environment* environment)
{
  mEnvironment = environment;
}

} // namespace OpenFDM
