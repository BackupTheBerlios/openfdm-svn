/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "LeafNode.h"
#include "NodeVisitor.h"

namespace OpenFDM {

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

} // namespace OpenFDM
