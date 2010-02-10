/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimAerosurfaceScale_H
#define OpenFDM_JSBSimAerosurfaceScale_H

#include "JSBSimFCSComponent.h"

#include <OpenFDM/BreakPointLookup.h>
#include <OpenFDM/SharedPtr.h>
#include <OpenFDM/Saturation.h>
#include <OpenFDM/Table.h>

namespace OpenFDM {

/// Just a small container mapping the JSBSim AerosurfaceScale parameters to
/// the OpenFDM models.
class JSBSimAerosurfaceScale :
    public JSBSimFCSComponent {
public:
  JSBSimAerosurfaceScale(const std::string& name);
  virtual ~JSBSimAerosurfaceScale(void);

  void setMinDomain(real_type minDomain);
  void setMaxDomain(real_type maxDomain);
  void setCentered(bool centered);

  void setMinValue(real_type minValue);
  void setMaxValue(real_type maxValue);

  void setGain(real_type gain);

private:
  SharedPtr<Saturation> mInputSaturation;
  SharedPtr<BreakPointLookup> mBreakPointLookup;
  SharedPtr<Table1D> mTable;
  real_type mGain;
};

} //namespace OpenFDM

#endif
