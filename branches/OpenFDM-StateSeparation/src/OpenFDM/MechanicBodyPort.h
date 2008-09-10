/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicBodyPort_H
#define OpenFDM_MechanicBodyPort_H

#include "MechanicAcceptorPortInfo.h"
#include "MechanicPortValue.h"
#include "SharedPtr.h"

namespace OpenFDM {

class MechanicBodyPort {
public:
  MechanicBodyPort(Node* node, const std::string& name) :
    mPort(new MechanicAcceptorPortInfo(node, name))
  {}
  MechanicPortValue* getPortValue(const PortValueVector& portValueVector) const
  {
    PortValue* portValue = mPort->getPortValue(portValueVector);
    OpenFDMAssert(portValue);
    OpenFDMAssert(dynamic_cast<MechanicPortValue*>(portValue));
    return static_cast<MechanicPortValue*>(portValue);
  }
  unsigned getPortIndex() const
  { return mPort->getIndex(); }
private:
  SharedPtr<MechanicAcceptorPortInfo> mPort;
};

} // namespace OpenFDM

#endif
