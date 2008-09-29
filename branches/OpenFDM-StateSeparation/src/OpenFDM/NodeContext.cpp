/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "NodeContext.h"

namespace OpenFDM {

NodeContext::NodeContext(const Node* node) :
  mNode(node)
{
  OpenFDMAssert(mNode);
}

NodeContext::~NodeContext()
{
}

const Node&
NodeContext::getNode() const
{
  return *mNode;
}

} // namespace OpenFDM
