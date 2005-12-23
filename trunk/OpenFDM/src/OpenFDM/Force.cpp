/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Force.h"

namespace OpenFDM {

Force::Force(const std::string& name, unsigned numParents) :
  Interact(name, numParents)
{
}

Force::~Force(void)
{
}

} // namespace OpenFDM
