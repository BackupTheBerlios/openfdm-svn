/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Port.h"

#include <string>
#include <vector>
#include <algorithm>

#include "LogStream.h"
#include "Object.h"
#include "Variant.h"
#include "Model.h"

namespace OpenFDM {

Port::Port(Node* node) :
  mNode(node)
{
}

Port::~Port()
{
}

void
Port::invalidate()
{
  removeAllConnections();
  mNode = 0;
}

WeakPtr<Node>
Port::getModel() const
{
  return mNode;
}

} // namespace OpenFDM
