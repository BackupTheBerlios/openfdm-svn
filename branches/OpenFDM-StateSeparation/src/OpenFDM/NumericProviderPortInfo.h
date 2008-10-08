/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NumericProviderPortInfo_H
#define OpenFDM_NumericProviderPortInfo_H

#include <string>
#include "ProviderPortInfo.h"
#include "NumericPortValue.h"
#include "Limits.h"

namespace OpenFDM {

class NumericProviderPortInfo : public ProviderPortInfo {
public:
  NumericProviderPortInfo(Node* node, const std::string& name, const Size& sz);
  virtual ~NumericProviderPortInfo();

  virtual unsigned getMaxConnects() const
  { return Limits<unsigned>::max(); }

  virtual bool acceptPortValue(const PortValue* portValue) const
  { return dynamic_cast<const NumericPortValue*>(portValue); }

protected:
  virtual NumericPortValue* newValueImplementation() const;
private:
  Size mSize;
};

} // namespace OpenFDM

#endif
