/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AirSpring_H
#define OpenFDM_AirSpring_H

#include "Model.h"
#include "Vector.h"
#include "LineForce.h"

namespace OpenFDM {

/// AirSpring
class AirSpring :
    public LineForce {
public:
  AirSpring(const std::string& name);
  virtual ~AirSpring(void);

  virtual void output(const TaskInfo& taskInfo);

  real_type getPushPressure(void) const;
  void setPushPressure(real_type pushPressure);

  real_type getPullPressure(void) const;
  void setPullPressure(real_type pullPressure);

  real_type getArea(void) const;
  void setArea(real_type area);

  real_type getMaxCompression(void) const;
  void setMaxCompression(real_type maxCompression);

  real_type getMinCompression(void) const;
  void setMinCompression(real_type minCompression);

  real_type getMaxDamperConstant(void) const;
  void setMaxDamperConstant(real_type maxDamperConstant);

  real_type getMinDamperConstant(void) const;
  void setMinDamperConstant(real_type minDamperConstant);

  real_type getGamma(void) const;
  void setGamma(real_type gamma);

private:
  real_type mPushPressure;
  real_type mPullPressure;

  real_type mArea;

  real_type mMaxCompression;
  real_type mMinCompression;

  real_type mMaxDamperConstant;
  real_type mMinDamperConstant;

  real_type mGamma;
};

} // namespace OpenFDM

#endif