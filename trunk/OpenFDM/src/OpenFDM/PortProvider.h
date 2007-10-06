/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_PortProvider_H
#define OpenFDM_PortProvider_H

#include "Port.h"

namespace OpenFDM {

class NumericPortAcceptor;
// class MechanicPortAcceptor;

class PortProvider :
    public Port {
public:
  PortProvider(Node* node);
  virtual ~PortProvider();

  virtual Port::ConnectResult addConnection(Connection* connection);
  virtual bool removeConnection(Connection* connection);

  virtual Port::ConnectResult provide(NumericPortAcceptor* port)
  { return Port::IncompatiblePort; }
  virtual bool unprovide(NumericPortAcceptor* port)
  { return false; }
};

} // namespace OpenFDM

#endif
