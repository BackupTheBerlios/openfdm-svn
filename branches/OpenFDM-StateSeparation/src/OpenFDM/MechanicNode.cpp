/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "MechanicNode.h"

#include "ConstNodeVisitor.h"
#include "MechanicContext.h"
#include "NodeVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(MechanicNode, LeafNode)
  END_OPENFDM_OBJECT_DEF

MechanicNode::MechanicNode(const std::string& name) :
  LeafNode(name)
{
}

MechanicNode::~MechanicNode()
{
}

void
MechanicNode::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
MechanicNode::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

} // namespace OpenFDM
