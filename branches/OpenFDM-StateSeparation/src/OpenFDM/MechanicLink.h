/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicLink_H
#define OpenFDM_MechanicLink_H

#include "MechanicLinkValue.h"
#include "PortInfo.h"
#include "SharedPtr.h"

namespace OpenFDM {

class MechanicLink {
public:
  MechanicLink(Node* node, const std::string& name) :
    mPort(new MechanicLinkInfo(node, name))
  {}
  MechanicLinkValue* getPortValue(const PortValueVector& portValueVector) const
  {
    PortValue* portValue = mPort->getPortValue(portValueVector);
    OpenFDMAssert(portValue);
    OpenFDMAssert(portValue->toMechanicLinkValue());
    return static_cast<MechanicLinkValue*>(portValue);
  }
  bool empty() const
  { return !mPort; }
  void clear()
  { if (!mPort) return; mPort->clear(); mPort = 0; }
  unsigned getPortIndex() const
  { OpenFDMAssert(mPort); return mPort->getIndex(); }
private:
  SharedPtr<MechanicLinkInfo> mPort;
};

} // namespace OpenFDM

#endif
