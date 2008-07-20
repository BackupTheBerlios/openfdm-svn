/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MatrixOutputPort_H
#define OpenFDM_MatrixOutputPort_H

#include "NumericProviderPortInfo.h"
#include "NumericPortValue.h"
#include "SharedPtr.h"

namespace OpenFDM {

class MatrixOutputPort {
public:
  MatrixOutputPort(Node* node, const std::string& name, const Size& size) :
    mPort(new NumericProviderPortInfo(node, name, size))
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
