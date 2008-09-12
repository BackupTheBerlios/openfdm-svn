/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "LeafNode.h"

#include "ConstNodeVisitor.h"
#include "NodeVisitor.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(LeafNode, Node)
  END_OPENFDM_OBJECT_DEF

LeafNode::LeafNode(const std::string& name) :
  Node(name)
{
}

LeafNode::~LeafNode()
{
}

void
LeafNode::accept(NodeVisitor& visitor)
{
  visitor.apply(*this);
}

void
LeafNode::accept(ConstNodeVisitor& visitor) const
{
  visitor.apply(*this);
}

} // namespace OpenFDM
