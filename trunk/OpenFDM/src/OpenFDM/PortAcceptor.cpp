/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Model.h"
#include "ModelGroup.h"
#include "Connection.h"
#include "PortAcceptor.h"

namespace OpenFDM {

PortAcceptor::PortAcceptor(Model* model) :
  Port(model)
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

} // namespace OpenFDM
