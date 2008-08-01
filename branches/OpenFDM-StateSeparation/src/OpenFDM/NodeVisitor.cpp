/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "NodeVisitor.h"

#include "LeafNode.h"

namespace OpenFDM {

NodeVisitor::~NodeVisitor()
{
}

void
NodeVisitor::apply(Node&)
{
}

void
NodeVisitor::apply(LeafNode& leafNode)
{
  apply(static_cast<Node&>(leafNode));
}

} // namespace OpenFDM
