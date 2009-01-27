/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RealInputPort_H
#define OpenFDM_RealInputPort_H

#include "NumericPortValue.h"
#include "Port.h"
#include "SharedPtr.h"

namespace OpenFDM {

class RealInputPort {
public:
  RealInputPort()
  { }
  RealInputPort(Node* node, const std::string& name, bool directInput) :
    mPort(new InputPort(node, name, Size(1, 1), directInput))
  { }
  NumericPortValue* getPortValue(const PortValueVector& portValueVector) const
  {
    OpenFDMAssert(mPort);
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
  { OpenFDMAssert(mPort); return mPort->getIndex(); }
  bool getDirectInput() const
  { OpenFDMAssert(mPort); return mPort->getDirectInput(); }
  void setDirectInput(bool directInput)
  { OpenFDMAssert(mPort); mPort->setDirectInput(directInput); }
private:
  SharedPtr<InputPort> mPort;
};

} // namespace OpenFDM

#endif
