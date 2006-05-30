/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Model.h"
#include "ModelGroup.h"
#include "Connection.h"
#include "PortProvider.h"

namespace OpenFDM {

PortProvider::PortProvider(Model* model) :
  Port(model)
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
  return connection->setPortProvider(0);
}

} // namespace OpenFDM
