/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Force.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Force, Interact)
  END_OPENFDM_OBJECT_DEF

// FIXME
BEGIN_OPENFDM_OBJECT_DEF(ExternalForce, Force)
  DEF_OPENFDM_PROPERTY(Vector3, Position, Serialized)
  DEF_OPENFDM_PROPERTY(Quaternion, Orientation, Serialized)
  END_OPENFDM_OBJECT_DEF

BEGIN_OPENFDM_OBJECT_DEF(InternalForce, Force)
  DEF_OPENFDM_PROPERTY(Vector3, Position0, Serialized)
  DEF_OPENFDM_PROPERTY(Quaternion, Orientation0, Serialized)
  DEF_OPENFDM_PROPERTY(Vector3, Position1, Serialized)
  DEF_OPENFDM_PROPERTY(Quaternion, Orientation1, Serialized)
  END_OPENFDM_OBJECT_DEF

BEGIN_OPENFDM_OBJECT_DEF(LineForce, InternalForce)
  END_OPENFDM_OBJECT_DEF

Force::Force(const std::string& name, unsigned numParents) :
  Interact(name, numParents)
{
}

Force::~Force(void)
{
}

} // namespace OpenFDM
