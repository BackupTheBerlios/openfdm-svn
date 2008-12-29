/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "DoubleLinkInteract.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(DoubleLinkInteract, Interact)
  END_OPENFDM_OBJECT_DEF

DoubleLinkInteract::DoubleLinkInteract(const std::string& name) :
  Interact(name),
  mMechanicLink0(this, "link0"),
  mMechanicLink1(this, "link1")
{
}

DoubleLinkInteract::~DoubleLinkInteract()
{
}

} // namespace OpenFDM
