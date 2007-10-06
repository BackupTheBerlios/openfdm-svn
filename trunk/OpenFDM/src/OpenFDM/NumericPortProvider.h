/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NumericPortProvider_H
#define OpenFDM_NumericPortProvider_H

#include "PortProvider.h"
#include "NumericPortAcceptor.h"

namespace OpenFDM {

class NumericPortProvider :
    public PortProvider {
public:
  NumericPortProvider(Node* node) : PortProvider(node) {}

  virtual Port::ConnectResult addConnection(Connection* connection)
  {
    Port::ConnectResult result = PortProvider::addConnection(connection);
    if (result != Port::Success)
      return result;
    mConnections.push_back(connection);
    return Port::Success;
  }
  virtual bool removeConnection(Connection* connection)
  {
    ConnectionList::iterator i = std::find(mConnections.begin(),
                                           mConnections.end(), connection);
    if (i == mConnections.end())
      return false;

    PortProvider::removeConnection(*i);
    mConnections.erase(i);
    return true;
  }
  virtual void removeAllConnections()
  {
    ConnectionList::iterator i = mConnections.begin();
    while (i != mConnections.end()) {
      PortProvider::removeConnection(*i);
      i = mConnections.erase(i);
    }
  }



  virtual Port::ConnectResult provide(NumericPortAcceptor* portAcceptor)
  {
    /// FIXME check if already connected ??
    Port::ConnectResult result = portAcceptor->setPortInterface(getPortInterface());
    if (result != Port::Success)
      return result;
    mPortAcceptors.push_back(portAcceptor);
    return result;
  }
  virtual bool unprovide(NumericPortAcceptor* port)
  {
    bool found = false;
    PortAcceptorList::iterator i;
    for (i = mPortAcceptors.begin(); i != mPortAcceptors.end();) {
      if (*i == port) {
        (*i)->setPortInterface(0);
        i = mPortAcceptors.erase(i);
        found = true;
      } else
        ++i;
    }
    return found;
  }

  PortInterface* getPortInterface() const
  { return mPortInterface; }

  Port::ConnectResult setPortInterface(PortInterface* portInterface)
  {
    Port::ConnectResult res = Port::Success;
    PortAcceptorList::iterator i;
    for (i = mPortAcceptors.begin(); i != mPortAcceptors.end(); ++i) {
      res = (*i)->setPortInterface(portInterface);
      if (!res)
        break;
    }
    if (i == mPortAcceptors.end()) {
      mPortInterface = portInterface;
      return Port::Success;
    }
    PortAcceptorList::iterator j;
    for (j = mPortAcceptors.begin(); j != i; ++j) {
      (*j)->setPortInterface(0);
    }
    return res;
  }

private:
  typedef std::list<SharedPtr<NumericPortAcceptor> > PortAcceptorList;
  typedef std::list<SharedPtr<Connection> > ConnectionList;

  PortAcceptorList mPortAcceptors;
  SharedPtr<PortInterface> mPortInterface;

  ConnectionList mConnections;
};

} // namespace OpenFDM

#endif
