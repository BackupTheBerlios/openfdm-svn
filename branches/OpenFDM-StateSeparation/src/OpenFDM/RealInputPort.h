/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_RealInputPort_H
#define OpenFDM_RealInputPort_H

#include "NumericAcceptorPortInfo.h"
#include "NumericPortValue.h"
#include "SharedPtr.h"

namespace OpenFDM {

class RealInputPort {
public:
  RealInputPort(Node* node, const std::string& name, bool directInput) :
    mPort(new NumericAcceptorPortInfo(node, name, Size(1, 1), directInput))
  {}
  NumericPortValue* getPortValue(const PortValueVector& portValueVector) const
  {
    PortValue* portValue = mPort->getPortValue(portValueVector);
    OpenFDMAssert(portValue);
    OpenFDMAssert(dynamic_cast<NumericPortValue*>(portValue));
    return static_cast<NumericPortValue*>(portValue);
  }
  unsigned getPortIndex() const
  { return mPort->getIndex(); }
  bool getDirectInput() const
  { return mPort->getDirectInput(); }
  void setDirectInput(bool directInput) const
  { mPort->setDirectInput(directInput); }
private:
  SharedPtr<NumericAcceptorPortInfo> mPort;
};

} // namespace OpenFDM

#endif
