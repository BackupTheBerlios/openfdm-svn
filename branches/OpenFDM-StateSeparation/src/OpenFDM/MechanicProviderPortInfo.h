/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicProviderPortInfo_H
#define OpenFDM_MechanicProviderPortInfo_H

#include <string>
#include "MechanicPortValue.h"
#include "ProviderPortInfo.h"

namespace OpenFDM {

class MechanicProviderPortInfo : public ProviderPortInfo {
public:
  MechanicProviderPortInfo(Node* node, const std::string& name) :
    ProviderPortInfo(node, name)
  { }
  virtual bool acceptPortValue(const PortValue* portValue) const
  { return dynamic_cast<const MechanicPortValue*>(portValue); }
protected:
  virtual MechanicPortValue* newValueImplementation() const
  { return new MechanicPortValue; }
};

} // namespace OpenFDM

#endif
