/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MatrixOutputPort_H
#define OpenFDM_MatrixOutputPort_H

#include "NumericPortValue.h"
#include "SharedPtr.h"

namespace OpenFDM {

class MatrixOutputPort {
public:
  MatrixOutputPort()
  { }
  MatrixOutputPort(Node* node, const std::string& name, const Size& size,
                   bool accelerationPort = false) :
    mPort(new OutputPort(node, name, size, accelerationPort))
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
