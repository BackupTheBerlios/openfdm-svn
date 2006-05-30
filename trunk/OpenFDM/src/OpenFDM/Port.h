/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Port_H
#define OpenFDM_Port_H

#include "Object.h"
#include "WeakPtr.h"

namespace OpenFDM {

class Model;
class Connection;

class Port :
    public Object {
public:
  enum ConnectResult {
    Success = 0,
    NoConnection, // The given Connection is a zero pointer
    Running, // The System is running, stop the System before manipulating it
    IsolatedModel, // Model does not belong to a parent Group
    DifferentGroups, // Model belong to different model groups
    StalePort, // Post still alive but originating model is dead
    AlreadyConnected, // This end of the connection is already connected
    IncompatiblePort, // The port is not compatible with the other end
    IncompatibelSize  // The port is not compatible with the other end
  };

  Port(Model* model);
  virtual ~Port();

  /// Interface to the /user/.
  /// Is implemented in PortAcceptor/PortProvider
  /// This function is responsible to distinguish into which end of the
  /// Connection object this Port belongs. This way the Connection object can
  /// make sure that it has exactly one PortProvider and exactly one
  /// PortAcceptor available to connect.
  virtual ConnectResult addConnection(Connection* connection) = 0;
  virtual bool removeConnection(Connection* connection) = 0;
  virtual void removeAllConnections() = 0;

  /// Sets the model it belongs to zero and cuts all connections
  void invalidate();

  /// Return the model this port belongs to
  WeakPtr<Model> getModel() const;

private:
  WeakPtr<Model> mModel;
};

} // namespace OpenFDM

#endif
