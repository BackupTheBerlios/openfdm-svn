/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "NodeInstance.h"

namespace OpenFDM {

NodeInstance::NodeInstance(const NodePath& nodePath,
                           const SampleTime& sampleTime, const Node* node) :
  AbstractNodeInstance(nodePath, sampleTime),
  mNodeContext(new NodeContext(node))
{
}

NodeInstance::~NodeInstance()
{
}

NodeContext&
NodeInstance::getNodeContext()
{
  return *mNodeContext;
}

const NodeContext&
NodeInstance::getNodeContext() const
{
  return *mNodeContext;
}

} // namespace OpenFDM
