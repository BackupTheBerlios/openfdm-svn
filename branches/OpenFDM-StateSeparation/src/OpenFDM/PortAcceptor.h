/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_PortAcceptor_H
#define OpenFDM_PortAcceptor_H

#include "Port.h"

namespace OpenFDM {

class PortProvider;

class PortAcceptor :
    public Port {
public:
  PortAcceptor(Node* node);
  virtual ~PortAcceptor();

  virtual ConnectResult addConnection(Connection* connection);
  virtual bool removeConnection(Connection* connection);

  virtual ConnectResult connect(PortProvider* port) = 0;
  virtual bool disconnect(PortProvider* port) = 0;
};

} // namespace OpenFDM

#endif
