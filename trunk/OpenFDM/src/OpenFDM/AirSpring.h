/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AirSpring_H
#define OpenFDM_AirSpring_H

#include "Model.h"
#include "Vector.h"

namespace OpenFDM {

/// AirSpring
class AirSpring :
    public Model {
public:
  AirSpring(const std::string& name);
  virtual ~AirSpring(void);

  virtual bool init(void);
  virtual void output(const TaskInfo& taskInfo);

  const real_type& getForce(void) const;

  const real_type& getPushPressure(void) const;
  void setPushPressure(const real_type& pushPressure);

  const real_type& getPullPressure(void) const;
  void setPullPressure(const real_type& pullPressure);

  const real_type& getArea(void) const;
  void setArea(const real_type& area);

  const real_type& getMaxCompression(void) const;
  void setMaxCompression(const real_type& maxCompression);

  const real_type& getMinCompression(void) const;
  void setMinCompression(const real_type& minCompression);

  const real_type& getMaxDamperConstant(void) const;
  void setMaxDamperConstant(const real_type& maxDamperConstant);

  const real_type& getMinDamperConstant(void) const;
  void setMinDamperConstant(const real_type& minDamperConstant);

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

  real_type mForce;

  /// The intput port which must provide the position
  RealPortHandle mPositionPort;
  /// The intput port which must provide the velocity
  RealPortHandle mVelocityPort;
};

} // namespace OpenFDM

#endif
