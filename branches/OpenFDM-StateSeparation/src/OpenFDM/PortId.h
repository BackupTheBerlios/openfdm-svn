/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_PortId_H
#define OpenFDM_PortId_H

#include "PortInfo.h"
#include "SharedPtr.h"
#include "WeakPtr.h"

namespace OpenFDM {

// Forward decl for friends ...
class Node;

class PortId {
public:
  PortId() {}
  // Hmm, FIXME, public??
  PortId(const SharedPtr<const PortInfo>& port) : _port(port) {}

  bool operator==(const PortId& other) const
  { return _port == other._port; }
  bool operator!=(const PortId& other) const
  { return _port != other._port; }
  bool operator<(const PortId& other) const
  { return _port < other._port; }

private:
  friend class Node;
  WeakPtr<const PortInfo> _port;
};

} // namespace OpenFDM

#endif
