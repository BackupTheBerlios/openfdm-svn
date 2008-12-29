/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "SingleLinkInteract.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(SingleLinkInteract, Interact)
  END_OPENFDM_OBJECT_DEF

SingleLinkInteract::SingleLinkInteract(const std::string& name) :
  Interact(name),
  mMechanicLink(this, "link")
{
}

SingleLinkInteract::~SingleLinkInteract()
{
}

} // namespace OpenFDM
