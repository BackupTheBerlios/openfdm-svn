/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RealOutputPort_H
#define OpenFDM_RealOutputPort_H

#include "NumericProviderPortInfo.h"
#include "NumericPortValue.h"
#include "SharedPtr.h"

namespace OpenFDM {

class RealOutputPort {
public:
  RealOutputPort(Node* node, const std::string& name) :
    mPort(new NumericProviderPortInfo(node, name, Size(1, 1)))
  { }
  NumericPortValue* getPortValue(const PortValueVector& portValueVector) const
  {
    PortValue* portValue = mPort->getPortValue(portValueVector);
    OpenFDMAssert(portValue);
    OpenFDMAssert(dynamic_cast<NumericPortValue*>(portValue));
    return static_cast<NumericPortValue*>(portValue);
  }
  unsigned getPortIndex() const
  { return mPort->getIndex(); }
private:
  SharedPtr<NumericProviderPortInfo> mPort;
};

} // namespace OpenFDM

#endif
