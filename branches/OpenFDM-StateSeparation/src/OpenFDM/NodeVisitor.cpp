/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "NodeVisitor.h"

#include "Group.h"
#include "GroupInterfaceNode.h"
#include "Input.h"
#include "Interact.h"
#include "Joint.h"
#include "LibraryNode.h"
#include "LeafNode.h"
#include "MechanicNode.h"
#include "AbstractModel.h"
#include "Output.h"
#include "RigidBody.h"
#include "RootJoint.h"
#include "Interact.h"

#include "PortInfo.h"

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
NodeVisitor::apply(GroupInterfaceNode& node)
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
NodeVisitor::apply(AbstractModel& node)
{
  apply(static_cast<LeafNode&>(node));
}

void
NodeVisitor::apply(Input& node)
{
  apply(static_cast<AbstractModel&>(node));
}

void
NodeVisitor::apply(Output& node)
{
  apply(static_cast<AbstractModel&>(node));
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
NodeVisitor::apply(Joint& node)
{
  apply(static_cast<MechanicNode&>(node));
}

void
NodeVisitor::apply(RootJoint& node)
{
  apply(static_cast<Joint&>(node));
}

void
NodeVisitor::apply(Interact& node)
{
  apply(static_cast<MechanicNode&>(node));
}

void
NodeVisitor::apply(const PortInfo& portInfo)
{
}

void
NodeVisitor::apply(const NumericPortInfo& portInfo)
{
  apply(static_cast<const PortInfo&>(portInfo));
}

void
NodeVisitor::apply(const MechanicLinkInfo& portInfo)
{
  apply(static_cast<const PortInfo&>(portInfo));
}

} // namespace OpenFDM
