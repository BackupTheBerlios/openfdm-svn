/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "Interact.h"
#include "NodeVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Interact, MechanicNode)
  END_OPENFDM_OBJECT_DEF

Interact::Interact(const std::string& name) :
  MechanicNode(name)
{
}

Interact::~Interact()
{
}

void
Interact::accept(NodeVisitor& visitor)
{
  visitor.apply(*this);
}


} // namespace OpenFDM
