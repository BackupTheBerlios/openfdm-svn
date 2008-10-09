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
class MechanicPortValue;

class PortValue : public Referenced {
public:
  virtual NumericPortValue* toNumericPortValue() { return 0; }
  virtual const NumericPortValue* toNumericPortValue() const { return 0; }
  virtual MechanicPortValue* toMechanicPortValue() { return 0; }
  virtual const MechanicPortValue* toMechanicPortValue() const { return 0; }

  static void destroy(const PortValue* portValue)
  { delete portValue; }

protected:
  virtual ~PortValue();
};

// FIXME Do I need a class for that???
typedef std::vector<SharedPtr<PortValue> > PortValueVector;

} // namespace OpenFDM

#endif
