/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DiscBrake_H
#define OpenFDM_DiscBrake_H

#include <string>

#include "Types.h"
#include "Model.h"

namespace OpenFDM {

/// Linear spring damper model
class DiscBrake :
    public Model {
public:
  DiscBrake(const std::string& name);
  virtual ~DiscBrake(void);

  virtual bool init(void);
  virtual void output(const TaskInfo& taskInfo);

  const real_type& getForce(void) const;

  real_type getFrictionConstant(void) const;
  void setFrictionConstant(real_type frictionConstant);

private:
  /// The friction constant for that viscosous friction model
  real_type mFrictionConstant;

  /// The output brake force
  real_type mForce;

  /// The intput port which must provide the position
  RealPortHandle mBrakePressurePort;
  /// The intput port which must provide the velocity
  RealPortHandle mVelocityPort;
};

} // namespace OpenFDM

#endif
