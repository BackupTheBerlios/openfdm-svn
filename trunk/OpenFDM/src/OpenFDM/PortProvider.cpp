/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "PortProvider.h"

#include "Model.h"
#include "ModelGroup.h"
#include "Connection.h"

namespace OpenFDM {

PortProvider::PortProvider(Node* node) :
  Port(node)
{
}

PortProvider::~PortProvider()
{
}

Port::ConnectResult
PortProvider::addConnection(Connection* connection)
{
  if (!connection)
    return Port::NoConnection;
  return connection->setPortProvider(this);
}

bool
PortProvider::removeConnection(Connection* connection)
{
  if (!connection)
    return false;
  connection->setPortProvider(0);
  return true;
}

} // namespace OpenFDM
