/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NumericProviderPortInfo_H
#define OpenFDM_NumericProviderPortInfo_H

#include <string>
#include "NumericPortValue.h"
#include "ProviderPortInfo.h"

namespace OpenFDM {

class NumericProviderPortInfo : public ProviderPortInfo {
public:
  NumericProviderPortInfo(Node* node, const std::string& name, const Size& size) :
    ProviderPortInfo(node, name),
    mSize(size)
  { }
protected:
  virtual NumericPortValue* newValueImplementation() const
  { return new NumericPortValue(mSize); }
private:
  Size mSize;
};

} // namespace OpenFDM

#endif
