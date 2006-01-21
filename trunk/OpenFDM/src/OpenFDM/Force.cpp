/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Force.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Force)
  END_OPENFDM_OBJECT_DEF

// FIXME
BEGIN_OPENFDM_OBJECT_DEF(ExternalForce)
  END_OPENFDM_OBJECT_DEF
BEGIN_OPENFDM_OBJECT_DEF(InternalForce)
  END_OPENFDM_OBJECT_DEF
BEGIN_OPENFDM_OBJECT_DEF(LineForce)
  END_OPENFDM_OBJECT_DEF

Force::Force(const std::string& name, unsigned numParents) :
  Interact(name, numParents)
{
}

Force::~Force(void)
{
}

} // namespace OpenFDM
