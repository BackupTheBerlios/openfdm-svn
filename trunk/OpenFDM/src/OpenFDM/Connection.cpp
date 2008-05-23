/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Connection.h"

#include "Port.h"
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
Connection::setPortProvider(PortProvider* portProvider)
{
  if (mPortProvider.lock() && portProvider)
    return Port::AlreadyConnected;

  if (!portProvider) {
    if (mPortAcceptor.lock() && mPortProvider.lock()) {
      mPortAcceptor.lock()->disconnect(mPortProvider.lock());
    } else {
      // FIXME
//       modelGroup->removeConnection(this);
    }
    mPortProvider.clear();
    return Port::Success;
  }

  SharedPtr<Node> providerModel = portProvider->getModel().lock();
  if (!providerModel)
    return Port::StalePort;
  if (!providerModel->getParent(0).lock())
    return Port::IsolatedModel;

  // if the other side is not yet connected, we are ok here
  if (!mPortAcceptor.lock()) {
    mPortProvider = portProvider;
    SharedPtr<ModelGroup> modelGroup = providerModel->getParent(0).lock();
    modelGroup->addConnection(this);
    return Port::Success;
  }

  SharedPtr<Node> acceptorModel = mPortAcceptor.lock()->getModel().lock();
  if (acceptorModel->getParent(0).lock() != providerModel->getParent(0).lock()) {
    mPortProvider.clear();
    return Port::DifferentGroups;
  }

  mPortProvider = portProvider;
  Port::ConnectResult result = mPortAcceptor.lock()->connect(mPortProvider.lock());
  if (result == Port::Success)
    return result;
  mPortProvider.clear();
  return result;
}

Port::ConnectResult
Connection::setPortAcceptor(PortAcceptor* portAcceptor)
{
  if (mPortAcceptor.lock() && portAcceptor)
    return Port::AlreadyConnected;

  if (!portAcceptor) {
    if (mPortAcceptor.lock() && mPortProvider.lock()) {
      mPortAcceptor.lock()->disconnect(mPortProvider.lock());
    } else {
      // FIXME
//       modelGroup->removeConnection(this);
    }
    mPortAcceptor.clear();
    return Port::Success;
  }
    
  SharedPtr<Node> acceptorModel = portAcceptor->getModel().lock();
  if (!acceptorModel)
    return Port::StalePort;
  SharedPtr<Node> acceptorParent = acceptorModel->getParent(0).lock(); 
  if (!acceptorParent)
    return Port::IsolatedModel;

  // if the other side is not yet connected, we are ok here
  if (!mPortProvider.lock()) {
    mPortAcceptor = portAcceptor;
    SharedPtr<ModelGroup> modelGroup = acceptorModel->getParent(0).lock();
    modelGroup->addConnection(this);

    return Port::Success;
  }

  SharedPtr<Node> providerModel = mPortProvider.lock()->getModel().lock();
  SharedPtr<Node> providerParent = providerModel->getParent(0).lock(); 
  if (acceptorParent != providerParent) {
    mPortAcceptor.clear();
    return Port::DifferentGroups;
  }

  mPortAcceptor = portAcceptor;
  Port::ConnectResult result = portAcceptor->connect(mPortProvider.lock());
  if (result == Port::Success)
    return result;
  mPortAcceptor.clear();
  return result;
}

Port::ConnectResult
Connection::connect(Port* port0, Port* port1)
{
  SharedPtr<Connection> connection = new Connection;
  Port::ConnectResult result = port0->addConnection(connection);
  if (result != Port::Success)
    return result;
  result = port1->addConnection(connection);
  return result;
}

Port::ConnectResult
Connection::connectRoute(PortProvider* port0, PortAcceptor* port1)
{
  Node::GroupPath path0;
  SharedPtr<Node> model = port0->getModel().lock();
  if (!model)
    return Port::IsolatedModel;
  path0 = model->getPath();

  Node::GroupPath path1;
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
  Node::GroupPath::reverse_iterator i;
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
  SharedPtr<PortProvider> portProvider = mPortProvider.lock();
  SharedPtr<PortAcceptor> portAcceptor = mPortAcceptor.lock();
  if (portProvider && portAcceptor)
    portAcceptor->disconnect(portProvider);
  mPortProvider.clear();
  mPortAcceptor.clear();
}

} // namespace OpenFDM
