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

  virtual bool canConnect(const PortInfo& portInfo) const
  { return portInfo.toAcceptorPortInfo(); }

  /// Dynamic casts
  virtual const ProviderPortInfo* toProviderPortInfo() const;
};

} // namespace OpenFDM

#endif
