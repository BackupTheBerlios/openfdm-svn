/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RealOutputPort_H
#define OpenFDM_RealOutputPort_H

#include "NumericPortValue.h"
#include "Port.h"
#include "SharedPtr.h"

namespace OpenFDM {

class RealOutputPort {
public:
  RealOutputPort()
  { }
  RealOutputPort(Node* node, const std::string& name,
                 bool accelerationPort = false) :
    mPort(new OutputPort(node, name, Size(1, 1), accelerationPort))
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
  SharedPtr<OutputPort> mPort;
};

} // namespace OpenFDM

#endif
