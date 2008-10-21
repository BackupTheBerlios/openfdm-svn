/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RealOutputPort_H
#define OpenFDM_RealOutputPort_H

#include "NumericPortValue.h"
#include "PortInfo.h"
#include "SharedPtr.h"

namespace OpenFDM {

class RealOutputPort {
public:
  RealOutputPort(Node* node, const std::string& name) :
    mPort(new OutputPortInfo(node, name, Size(1, 1)))
  { }
  NumericPortValue* getPortValue(const PortValueVector& portValueVector) const
  {
    PortValue* portValue = mPort->getPortValue(portValueVector);
    OpenFDMAssert(portValue);
    OpenFDMAssert(portValue->toNumericPortValue());
    return static_cast<NumericPortValue*>(portValue);
  }
  bool empty() const
  { return !mPort; }
  void clear()
  { if (!mPort) return; mPort->clear(); mPort = 0; }
  unsigned getPortIndex() const
  { return mPort->getIndex(); }
private:
  SharedPtr<OutputPortInfo> mPort;
};

} // namespace OpenFDM

#endif
