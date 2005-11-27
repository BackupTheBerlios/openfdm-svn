/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimKinemat_H
#define OpenFDM_JSBSimKinemat_H

#include "JSBSimFCSComponent.h"

namespace OpenFDM {

/// Just a small container mapping the JSBSim Kinemat parameters to
/// the OpenFDM models.
class JSBSimKinemat :
    public JSBSimFCSComponent {
public:
  JSBSimKinemat(const std::string& name);
  virtual ~JSBSimKinemat(void);

  void setRateLimit(real_type rateLimit);
  void setMinValue(real_type minValue);
  void setMaxValue(real_type maxValue);
  void setNoScale(bool noScale);

private:
  bool mNoScale;
  shared_ptr<Gain> mInputGain;
  shared_ptr<Saturation> mInputSaturation;
  shared_ptr<Saturation> mKinematRateLimit;
  shared_ptr<Gain> mOutputNormGain;
};

} //namespace OpenFDM

#endif
