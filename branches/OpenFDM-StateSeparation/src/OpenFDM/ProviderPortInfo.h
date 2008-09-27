/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ProviderPortInfo_H
#define OpenFDM_ProviderPortInfo_H

#include <string>
#include "PortInfo.h"

namespace OpenFDM {

class ProviderPortInfo : public PortInfo {
public:
  ProviderPortInfo(Node* node, const std::string& name);
  virtual ~ProviderPortInfo();

  /// Public interface to instantiate a new port value
  PortValue* newValue() const;

  /// Dynamic casts
  virtual const ProviderPortInfo* toProviderPortInfo() const;

protected:
  /// The basic property of a provider port is that it can provide
  /// a port value that can be used by acceptor ports.
  /// It s just important to know that one of the connected ports
  /// is able to provide storage for the value.
  virtual PortValue* newValueImplementation() const = 0;
};

} // namespace OpenFDM

#endif
