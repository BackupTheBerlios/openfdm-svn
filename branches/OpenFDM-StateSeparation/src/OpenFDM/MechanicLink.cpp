/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#include "MechanicLink.h"

#include "ConstNodeVisitor.h"
#include "NodeVisitor.h"

namespace OpenFDM {

MechanicLink::MechanicLink(Node* node, const std::string& name) :
  Port(node, name)
{
}

MechanicLink::~MechanicLink()
{
}

void
MechanicLink::accept(NodeVisitor& visitor) const
{
  visitor.apply(*this);
}

void
MechanicLink::accept(ConstNodeVisitor& visitor) const
{
  visitor.apply(*this);
}

const MechanicLink*
MechanicLink::toMechanicLink() const
{
  return this;
}

} // namespace OpenFDM
