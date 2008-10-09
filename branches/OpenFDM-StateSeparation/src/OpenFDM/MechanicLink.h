/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicLink_H
#define OpenFDM_MechanicLink_H

#include "MechanicPortValue.h"
#include "PortInfo.h"
#include "SharedPtr.h"

namespace OpenFDM {

class MechanicLink {
public:
  MechanicLink(Node* node, const std::string& name) :
    mPort(new MechanicLinkInfo(node, name))
  {}
  MechanicPortValue* getPortValue(const PortValueVector& portValueVector) const
  {
    PortValue* portValue = mPort->getPortValue(portValueVector);
    OpenFDMAssert(portValue);
    OpenFDMAssert(portValue->toMechanicPortValue());
    return static_cast<MechanicPortValue*>(portValue);
  }
  unsigned getPortIndex() const
  { return mPort->getIndex(); }
private:
  SharedPtr<MechanicLinkInfo> mPort;
};

} // namespace OpenFDM

#endif
