/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "PortInfo.h"
#include "Node.h"

namespace OpenFDM {

PortInfo::PortInfo(Node* node, const std::string& name) :
  mNode(node),
  mName(name),
  mIndex(~0u)
{
  OpenFDMAssert(node);
  node->addPort(this);
}

PortInfo::~PortInfo()
{
  SharedPtr<Node> node = mNode.lock();
  if (node)
    node->removePort(this);
}

void
PortInfo::setName(const std::string& name)
{
  mName = name;
}

} // namespace OpenFDM
