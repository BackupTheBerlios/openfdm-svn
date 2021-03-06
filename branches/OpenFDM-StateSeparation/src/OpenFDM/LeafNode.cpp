/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "LeafNode.h"

#include "ConstNodeVisitor.h"
#include "NodeVisitor.h"
#include "TypeInfo.h"
#include "Variant.h"

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
  visitor.handleNodePathAndApply(this);
}

void
LeafNode::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

} // namespace OpenFDM
