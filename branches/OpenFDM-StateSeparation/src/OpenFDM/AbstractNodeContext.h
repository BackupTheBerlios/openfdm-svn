/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AbstractNodeContext_H
#define OpenFDM_AbstractNodeContext_H

#include "Referenced.h"

namespace OpenFDM {

class Node;
class Port;
class PortValue;

class AbstractNodeContext : public Referenced {
public:
  AbstractNodeContext();
  virtual ~AbstractNodeContext();

  /// Returns the Node it belongs to.
  virtual const Node& getNode() const = 0;

  /// Port value accessors for System external usage.
  virtual const PortValue* getPortValue(const Port&) const = 0;
};

} // namespace OpenFDM

#endif
