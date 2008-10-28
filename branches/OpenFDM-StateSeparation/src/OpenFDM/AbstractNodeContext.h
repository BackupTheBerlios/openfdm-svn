/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AbstractNodeContext_H
#define OpenFDM_AbstractNodeContext_H

#include "Node.h"
#include "PortValueList.h"

namespace OpenFDM {

class AbstractNodeContext : public Referenced {
public:
  AbstractNodeContext();
  virtual ~AbstractNodeContext();

  /// Returns the Node it belongs to.
  virtual const Node& getNode() const = 0;

  /// Port value accessors for System external usage.
  virtual const PortValue* getPortValue(const PortInfo& portInfo) const;

  /// Set port value for the given port.
  virtual void setPortValue(const PortInfo& portInfo, PortValue* portValue);

  /// might vanish???
  PortValueList& getPortValueList()
  { return mPortValueList; }
  const PortValueList& getPortValueList() const
  { return mPortValueList; }

  // PortValues
  PortValueList mPortValueList;
};

} // namespace OpenFDM

#endif
