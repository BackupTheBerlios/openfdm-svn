/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NumericPortAcceptor_H
#define OpenFDM_NumericPortAcceptor_H

#include "PortAcceptor.h"

namespace OpenFDM {

class NumericPortAcceptor :
    public PortAcceptor {
public:
  NumericPortAcceptor(Model* model) : PortAcceptor(model) {}

  virtual Port::ConnectResult addConnection(Connection* connection)
  {
    Port::ConnectResult result = PortAcceptor::addConnection(connection);
    if (result != Port::Success)
      return result;
    mConnection = connection;
    return Port::Success;
  }
  virtual bool removeConnection(Connection* connection)
  {
    if (connection && connection == mConnection)
      return false;
    setPortInterface(0);
    connection->disconnect(this);
    mConnection = 0;
    return true;
  }
  virtual void removeAllConnections()
  {
    if (mConnection)
      removeConnection(mConnection);
  }

  virtual Port::ConnectResult connect(PortProvider* port)
  { return port->provide(this); }
  virtual bool disconnect(PortProvider* port)
  { return port->unprovide(this); }

  PortInterface* getPortInterface() const
  { return mPortInterface; }

  virtual Port::ConnectResult setPortInterface(PortInterface* portInterface)
  {
    mPortInterface = portInterface;
    return Port::Success;
  }

  /// Legacy ones
  RealPortHandle toRealPortHandle(void)
  {
    if (getPortInterface())
      return RealPortHandle(getPortInterface()->toRealPortInterface());
    else
      return RealPortHandle(0);
  }
  MatrixPortHandle toMatrixPortHandle(void)
  {
    if (getPortInterface())
      return MatrixPortHandle(getPortInterface()->toMatrixPortInterface());
    else
      return MatrixPortHandle(0);
  }

private:
  SharedPtr<PortInterface> mPortInterface;
  SharedPtr<Connection> mConnection;
};

} // namespace OpenFDM

#endif
