/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#include "Port.h"

#include "ConstNodeVisitor.h"
#include "Node.h"
#include "NodeVisitor.h"

namespace OpenFDM {

Port::Port(Node* node, const std::string& name) :
  mNode(node),
  mName(name),
  mIndex(~0u)
{
  OpenFDMAssert(node);
  node->addPort(this);
}

Port::~Port()
{
  clear();
}

void
Port::accept(NodeVisitor& visitor) const
{
  visitor.apply(*this);
}

void
Port::accept(ConstNodeVisitor& visitor) const
{
  visitor.apply(*this);
}

void
Port::setName(const std::string& name)
{
  mName = name;
}

void
Port::clear()
{
  SharedPtr<Node> node = mNode.lock();
  if (!node)
    return;
  node->removePort(this);
}



void
NumericPort::accept(NodeVisitor& visitor) const
{
  visitor.apply(*this);
}

void
NumericPort::accept(ConstNodeVisitor& visitor) const
{
  visitor.apply(*this);
}

} // namespace OpenFDM
