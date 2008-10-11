/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "ConstNodeVisitor.h"

#include "Group.h"
#include "Interact.h"
#include "LibraryNode.h"
#include "LeafNode.h"
#include "MechanicNode.h"
#include "Model.h"
#include "Input.h"
#include "Output.h"
#include "RigidBody.h"
#include "RootJoint.h"
#include "Interact.h"

namespace OpenFDM {

ConstNodeVisitor::~ConstNodeVisitor()
{
}

void
ConstNodeVisitor::apply(const Node&)
{
}

void
ConstNodeVisitor::apply(const Group& node)
{
  apply(static_cast<const Node&>(node));
}

void
ConstNodeVisitor::apply(const GroupInterfaceNode& node)
{
  apply(static_cast<const Node&>(node));
}

void
ConstNodeVisitor::apply(const LibraryNode& libraryNode)
{
  apply(static_cast<const Node&>(libraryNode));
}

void
ConstNodeVisitor::apply(const LeafNode& leafNode)
{
  apply(static_cast<const Node&>(leafNode));
}

void
ConstNodeVisitor::apply(const Model& node)
{
  apply(static_cast<const LeafNode&>(node));
}

void
ConstNodeVisitor::apply(const Input& node)
{
  apply(static_cast<const Model&>(node));
}

void
ConstNodeVisitor::apply(const Output& node)
{
  apply(static_cast<const Model&>(node));
}

void
ConstNodeVisitor::apply(const MechanicNode& node)
{
  apply(static_cast<const LeafNode&>(node));
}

void
ConstNodeVisitor::apply(const RigidBody& node)
{
  apply(static_cast<const MechanicNode&>(node));
}

void
ConstNodeVisitor::apply(const Interact& node)
{
  apply(static_cast<const MechanicNode&>(node));
}

void
ConstNodeVisitor::apply(const RootJoint& node)
{
  apply(static_cast<const Interact&>(node));
}

} // namespace OpenFDM
