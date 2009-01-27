/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicLink_H
#define OpenFDM_MechanicLink_H

#include "MechanicLinkValue.h"
#include "Port.h"
#include "SharedPtr.h"

namespace OpenFDM {

class MechanicLink_ {
public:
  MechanicLink_(Node* node, const std::string& name) :
    mPort(new MechanicLink(node, name))
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
  SharedPtr<MechanicLink> mPort;
};

} // namespace OpenFDM

#endif
