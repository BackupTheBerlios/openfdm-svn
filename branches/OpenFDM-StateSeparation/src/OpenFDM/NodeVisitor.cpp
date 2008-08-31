/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "NodeVisitor.h"

#include "LeafNode.h"
#include "Model.h"
#include "Group.h"

namespace OpenFDM {

NodeVisitor::~NodeVisitor()
{
}

void
NodeVisitor::apply(Node&)
{
}

void
NodeVisitor::apply(Group& node)
{
  apply(static_cast<Node&>(node));
}

void
NodeVisitor::apply(GroupAcceptorNode& node)
{
  apply(static_cast<Node&>(node));
}

void
NodeVisitor::apply(GroupProviderNode& node)
{
  apply(static_cast<Node&>(node));
}

void
NodeVisitor::apply(LeafNode& leafNode)
{
  apply(static_cast<Node&>(leafNode));
}

void
NodeVisitor::apply(Model& node)
{
  apply(static_cast<LeafNode&>(node));
}

} // namespace OpenFDM
