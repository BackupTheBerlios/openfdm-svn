/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "PortAcceptor.h"

#include "Model.h"
#include "ModelGroup.h"
#include "Connection.h"

namespace OpenFDM {

PortAcceptor::PortAcceptor(Node* node) :
  Port(node)
{
}

PortAcceptor::~PortAcceptor()
{
}

Port::ConnectResult
PortAcceptor::addConnection(Connection* connection)
{
  if (!connection)
    return Port::NoConnection;
  return connection->setPortAcceptor(this);
}

bool
PortAcceptor::removeConnection(Connection* connection)
{
  if (!connection)
    return false;
  connection->setPortAcceptor(0);
  return true;
}

} // namespace OpenFDM
