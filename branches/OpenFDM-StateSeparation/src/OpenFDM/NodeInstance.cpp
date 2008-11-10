/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "NodeInstance.h"

namespace OpenFDM {

NodeInstance::NodeInstance(const SampleTime& sampleTime, const Node* node) :
  AbstractNodeInstance(sampleTime),
  mNode(node)
{
}

NodeInstance::~NodeInstance()
{
}

} // namespace OpenFDM
