/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#include "DoubleLinkInteract.h"

#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(DoubleLinkInteract, Interact)
  END_OPENFDM_OBJECT_DEF

DoubleLinkInteract::DoubleLinkInteract(const std::string& name) :
  Interact(name),
  mMechanicLink0(new MechanicLink(this, "link0")),
  mMechanicLink1(new MechanicLink(this, "link1"))
{
}

DoubleLinkInteract::~DoubleLinkInteract()
{
}

} // namespace OpenFDM
