/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Object.h"
#include "Environment.h"
#include "EnvironmentObject.h"

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
