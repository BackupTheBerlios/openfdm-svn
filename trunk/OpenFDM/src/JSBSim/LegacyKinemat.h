/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_LegacyKinemat_H
#define OpenFDM_LegacyKinemat_H

#include <OpenFDM/ModelGroup.h>

namespace OpenFDM {

/// Just a small container mapping the JSBSim Kinemat parameters to
/// the OpenFDM models.
class LegacyKinemat :
    public ModelGroup {
public:
  LegacyKinemat(const std::string& name);
  virtual ~LegacyKinemat(void);

  void setRateLimit(real_type rateLimit);
  void setMinValue(real_type minValue);
  void setMaxValue(real_type maxValue);
  void setNoScale(bool noScale);

private:
  bool mNoScale;
  shared_ptr<Gain> mInputGain;
  shared_ptr<Saturation> mInputSaturation;
  shared_ptr<Saturation> mKinematRateLimit;
  shared_ptr<Gain> mOutputGain;
};

} //namespace OpenFDM

#endif
