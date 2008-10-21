/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MatrixInputPort_H
#define OpenFDM_MatrixInputPort_H

#include "NumericPortValue.h"
#include "SharedPtr.h"

namespace OpenFDM {

class MatrixInputPort {
public:
  MatrixInputPort()
  { }
  MatrixInputPort(Node* node, const std::string& name,
                  const Size& size, bool directInput) :
    mPort(new InputPortInfo(node, name, size, directInput))
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
  void setDirectInput(bool directInput) const
  { OpenFDMAssert(mPort); mPort->setDirectInput(directInput); }
private:
  SharedPtr<InputPortInfo> mPort;
};

} // namespace OpenFDM

#endif
