/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "PortInfo.h"

#include "ConstNodeVisitor.h"
#include "Node.h"
#include "NodeVisitor.h"

namespace OpenFDM {

PortInfo::PortInfo(Node* node, const std::string& name) :
  mNode(node),
  mName(name),
  mIndex(~0u),
  mOptional(false)
{
  OpenFDMAssert(node);
  node->addPort(this);
}

PortInfo::~PortInfo()
{
  clear();
}

void
PortInfo::accept(NodeVisitor& visitor) const
{
  visitor.apply(*this);
}

void
PortInfo::accept(ConstNodeVisitor& visitor) const
{
  visitor.apply(*this);
}

void
PortInfo::setName(const std::string& name)
{
  mName = name;
}

void
PortInfo::clear()
{
  SharedPtr<Node> node = mNode.lock();
  if (!node)
    return;
  node->removePort(this);
}



void
NumericPortInfo::accept(NodeVisitor& visitor) const
{
  visitor.apply(*this);
}

void
NumericPortInfo::accept(ConstNodeVisitor& visitor) const
{
  visitor.apply(*this);
}

void
MechanicLinkInfo::accept(NodeVisitor& visitor) const
{
  visitor.apply(*this);
}

void
MechanicLinkInfo::accept(ConstNodeVisitor& visitor) const
{
  visitor.apply(*this);
}

} // namespace OpenFDM
