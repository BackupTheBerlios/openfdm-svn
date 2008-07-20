/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MatrixInputPort_H
#define OpenFDM_MatrixInputPort_H

#include "NumericAcceptorPortInfo.h"
#include "NumericPortValue.h"
#include "SharedPtr.h"

namespace OpenFDM {

class MatrixInputPort {
public:
  MatrixInputPort(Node* node, const std::string& name, const Size& size) :
    mPort(new NumericAcceptorPortInfo(node, name, size))
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
private:
  SharedPtr<NumericAcceptorPortInfo> mPort;
};

} // namespace OpenFDM

#endif
