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
  RealOutputPort()
  { }
  RealOutputPort(Node* node, const std::string& name,
                 bool accelerationPort = false) :
    mPort(new OutputPortInfo(node, name, Size(1, 1), accelerationPort))
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
  bool getAccelerationOutput() const
  { OpenFDMAssert(mPort); return mPort->getAccelerationOutput(); }
  void setAccelerationOutput(bool accelerationOutput)
  { OpenFDMAssert(mPort); mPort->setAccelerationOutput(accelerationOutput); }
private:
  SharedPtr<OutputPortInfo> mPort;
};

} // namespace OpenFDM

#endif
