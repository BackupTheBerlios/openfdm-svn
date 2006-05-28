/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Port.h"
#include "Connection.h"
#include "ModelGroup.h"
#include "GroupInput.h"
#include "GroupOutput.h"

namespace OpenFDM {

Connection::Connection(const std::string& name) :
  Object(name)
{
}

Connection::~Connection()
{
  // make sure there is nothing left ...
  disconnect();
}

Port::ConnectResult
Connection::connect(Port* port)
{
  if (!port)
    return Port::NoPort;
  return port->addConnection(this);
}

bool
Connection::disconnect(Port* port)
{
  if (!port)
    return false;
  
  if (port == mPortProvider) {
    if (mPortAcceptor)
      mPortAcceptor->disconnect(mPortProvider);
    mPortProvider = 0;
    return true;
  }
  if (port == mPortAcceptor) {
    if (mPortProvider)
      mPortAcceptor->disconnect(mPortProvider);
    mPortAcceptor = 0;
    return true;
  }
  return false;
}

Port::ConnectResult
Connection::setPortProvider(PortProvider* portProvider)
{
  if (mPortProvider && portProvider)
    return Port::AlreadyConnected;

  SharedPtr<Model> providerModel = portProvider->getModel().lock();
  if (!providerModel)
    return Port::StalePort;
  if (!providerModel->getParent())
    return Port::IsolatedModel;

  // if the other side is not yet connected, we are ok here
  if (!mPortAcceptor) {
    mPortProvider = portProvider;
    ModelGroup* modelGroup = providerModel->getParent();
    modelGroup->addConnection(this);

    return Port::Success;
  }

  SharedPtr<Model> acceptorModel = mPortAcceptor->getModel().lock();
  if (acceptorModel->getParent() != providerModel->getParent()) {
    mPortProvider = 0;
    return Port::DifferentGroups;
  }

  mPortProvider = portProvider;
  Port::ConnectResult result = mPortAcceptor->connect(mPortProvider);
  if (result == Port::Success)
    return result;
  mPortProvider = 0;
  return result;
}

Port::ConnectResult
Connection::setPortAcceptor(PortAcceptor* portAcceptor)
{
  if (mPortAcceptor && portAcceptor)
    return Port::AlreadyConnected;

  SharedPtr<Model> acceptorModel = portAcceptor->getModel().lock();
  if (!acceptorModel)
    return Port::StalePort;
  if (!acceptorModel->getParent())
    return Port::IsolatedModel;

  // if the other side is not yet connected, we are ok here
  if (!mPortProvider) {
    mPortAcceptor = portAcceptor;
    ModelGroup* modelGroup = acceptorModel->getParent();
    modelGroup->addConnection(this);

    return Port::Success;
  }

  SharedPtr<Model> providerModel = mPortProvider->getModel().lock();
  if (acceptorModel->getParent() != providerModel->getParent()) {
    mPortAcceptor = 0;
    return Port::DifferentGroups;
  }

  mPortAcceptor = portAcceptor;
  Port::ConnectResult result = mPortAcceptor->connect(mPortProvider);
  if (result == Port::Success)
    return result;
  mPortAcceptor = 0;
  return result;
}

Port::ConnectResult
Connection::connect(Port* port0, Port* port1)
{
  SharedPtr<Connection> connection = new Connection;
  Port::ConnectResult result = connection->connect(port0);
  if (result != Port::Success)
    return result;
  result = connection->connect(port1);
  return result;
}

Port::ConnectResult
Connection::connectRoute(PortProvider* port0, PortAcceptor* port1)
{
  Model::Path path0;
  SharedPtr<Model> model = port0->getModel().lock();
  if (!model)
    return Port::IsolatedModel;
  path0 = model->getPath();

  Model::Path path1;
  model = port1->getModel().lock();
  if (!model)
    return Port::IsolatedModel;
  path1 = model->getPath();

  if (path0.front() != path1.front())
    return Port::DifferentGroups;
  
  while (path0.front() == path1.front()) {
    path0.pop_front();
    path1.pop_front();
  }
  
  // Ok, now the paths are clear
  Model::Path::reverse_iterator i;
  i = path0.rbegin();
  while (i != path0.rend()) {
    SharedPtr<GroupOutput> groupOutput = new GroupOutput(port0->getName());
    (*i)->addModel(groupOutput, true);
    Port::ConnectResult result = connect(port0, groupOutput->getInputPort(0));
    if (result != Port::Success)
      return result;
    port0 = groupOutput->getGroupOutput();
    ++i;
  }
  
  i = path1.rbegin();
  while (i != path1.rend()) {
    SharedPtr<GroupInput> groupInput = new GroupInput(port1->getName());
    (*i)->addModel(groupInput, true);
    Port::ConnectResult result = connect(groupInput->getOutputPort(0), port1);
    if (result != Port::Success)
      return result;
    port1 = groupInput->getGroupInput();
    ++i;
  }

  return connect(port0, port1);
}

void
Connection::disconnect()
{
  if (mPortProvider && mPortAcceptor)
    mPortAcceptor->disconnect(mPortProvider);
  mPortProvider = 0;
  mPortAcceptor = 0;
}

} // namespace OpenFDM