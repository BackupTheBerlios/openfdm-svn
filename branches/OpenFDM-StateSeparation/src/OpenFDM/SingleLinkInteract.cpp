/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#include "SingleLinkInteract.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(SingleLinkInteract, Interact)
  DEF_OPENFDM_PROPERTY(Vector3, Position, Serialized)
  END_OPENFDM_OBJECT_DEF

SingleLinkInteract::SingleLinkInteract(const std::string& name) :
  Interact(name),
  mMechanicLink(new MechanicLink(this, "link")),
  mPosition(0, 0, 0)
{
}

SingleLinkInteract::~SingleLinkInteract()
{
}

void
SingleLinkInteract::setPosition(const Vector3& position)
{
  mPosition = position;
}

const Vector3&
SingleLinkInteract::getPosition() const
{
  return mPosition;
}

} // namespace OpenFDM
