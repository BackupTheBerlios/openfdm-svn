/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "NodeVisitor.h"

#include "Group.h"
#include "Interact.h"
#include "LibraryNode.h"
#include "LeafNode.h"
#include "MechanicNode.h"
#include "Model.h"
#include "Output.h"
#include "RigidBody.h"
#include "RootJoint.h"
#include "Interact.h"

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
NodeVisitor::apply(GroupInput& node)
{
  apply(static_cast<Node&>(node));
}

void
NodeVisitor::apply(GroupOutput& node)
{
  apply(static_cast<Node&>(node));
}

void
NodeVisitor::apply(GroupMechanicLink& node)
{
  apply(static_cast<Node&>(node));
}

void
NodeVisitor::apply(LibraryNode& libraryNode)
{
  apply(static_cast<Node&>(libraryNode));
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

void
NodeVisitor::apply(Output& node)
{
  apply(static_cast<Model&>(node));
}

void
NodeVisitor::apply(MechanicNode& node)
{
  apply(static_cast<LeafNode&>(node));
}

void
NodeVisitor::apply(RigidBody& node)
{
  apply(static_cast<MechanicNode&>(node));
}

void
NodeVisitor::apply(Interact& node)
{
  apply(static_cast<MechanicNode&>(node));
}

void
NodeVisitor::apply(RootJoint& node)
{
  apply(static_cast<Interact&>(node));
}

} // namespace OpenFDM
