/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MatrixOutputPort_H
#define OpenFDM_MatrixOutputPort_H

#include "NumericPortValue.h"
#include "SharedPtr.h"

namespace OpenFDM {

class MatrixOutputPort {
public:
  MatrixOutputPort(Node* node, const std::string& name, const Size& size) :
    mPort(new OutputPortInfo(node, name, size))
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
