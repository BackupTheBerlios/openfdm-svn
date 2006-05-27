/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Connection_H
#define OpenFDM_Connection_H

#include "Object.h"
#include "SharedPtr.h"
#include "PortAcceptor.h"
#include "PortProvider.h"

namespace OpenFDM {

/// Class representing an arbitrary connection between two Models.
/// Each connection can connect a PortProvider with a PortAcceptor.
/// Both connection ends must connect models attached to the same ModelGroup.
/// The Models must be already attached to a ModelGroup before they can be
/// connected.

class Connection :
    public Object {
public:
  /// Each connection can have its own name
  Connection(const std::string& name = std::string());
  virtual ~Connection();

  /// Connect this connection to the given Port
  Port::ConnectResult connect(Port* port);
  /// Disconnect this connection from the given Port
  bool disconnect(Port* port);

  /// Set the PortProvider end.
  /// Tries to complete the connection if the other end is already connected.
  Port::ConnectResult setPortProvider(PortProvider* portProvider);
  /// Set the PortAcceptor end
  /// Tries to complete the connection if the other end is already connected.
  Port::ConnectResult setPortAcceptor(PortAcceptor* portAcceptor);

  /// Return the PortProvider of this connection
  const PortProvider* getPortProvider() const
  { return mPortProvider; }
  /// Return the PortAcceptor of this connection
  const PortAcceptor* getPortAcceptor() const
  { return mPortAcceptor; }

  /// Connect the two given ports, zero checks are done.
  /// The two ports must belong to models of the same group.
  static Port::ConnectResult connect(Port* port0, Port* port1);
  /// Connect the two given ports, zero checks are done.
  /// The two ports must just belong to models of the same system.
  /// If required ModelGroup inputs or outputs are inserted.
  static Port::ConnectResult connectRoute(PortProvider* port0, PortAcceptor* port1);

private:
  /// Disconnect this connection from all attached Ports
  void disconnect();

  WeakPtr<PortProvider> mPortProvider;
  WeakPtr<PortAcceptor> mPortAcceptor;
};

} // namespace OpenFDM

#endif
