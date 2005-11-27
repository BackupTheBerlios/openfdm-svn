/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimAerosurfaceScale_H
#define OpenFDM_JSBSimAerosurfaceScale_H

#include "JSBSimFCSComponent.h"

namespace OpenFDM {

class Table1D;

/// Just a small container mapping the JSBSim AerosurfaceScale parameters to
/// the OpenFDM models.
class JSBSimAerosurfaceScale :
    public JSBSimFCSComponent {
public:
  JSBSimAerosurfaceScale(const std::string& name);
  virtual ~JSBSimAerosurfaceScale(void);

  void setMinValue(real_type minValue);
  void setMaxValue(real_type maxValue);

private:
  shared_ptr<Table1D> mTable;
};

} //namespace OpenFDM

#endif
