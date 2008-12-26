/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_PortValue_H
#define OpenFDM_PortValue_H

#include <vector>
#include "Referenced.h"
#include "SharedPtr.h"

namespace OpenFDM {

class NumericPortValue;
class MechanicLinkValue;

class PortValue : public Referenced {
public:
  virtual NumericPortValue* toNumericPortValue();
  virtual const NumericPortValue* toNumericPortValue() const;
  virtual MechanicLinkValue* toMechanicLinkValue();
  virtual const MechanicLinkValue* toMechanicLinkValue() const;

  static void destroy(const PortValue* portValue);

protected:
  virtual ~PortValue();
};

// FIXME Do I need a class for that???
typedef std::vector<SharedPtr<PortValue> > PortValueVector;

} // namespace OpenFDM

#endif
