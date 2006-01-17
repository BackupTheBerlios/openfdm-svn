/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DiscBrake_H
#define OpenFDM_DiscBrake_H

#include <string>

#include "Types.h"
#include "Model.h"

namespace OpenFDM {

/// A modified Dahl fricion model
class DiscBrake : public Model {
  OPENFDM_OBJECT(DiscBrake, Model);
public:
  DiscBrake(const std::string& name);
  virtual ~DiscBrake(void);

  virtual bool init(void);
  virtual void output(const TaskInfo& taskInfo);

  virtual void setState(const StateStream& state);
  virtual void getState(StateStream& state) const;
  virtual void getStateDeriv(StateStream& stateDeriv);

  const real_type& getForce(void) const;

  const real_type& getMinForce(void) const;
  void setMinForce(const real_type& minForce);

  const real_type& getMaxForce(void) const;
  void setMaxForce(const real_type& maxForce);

private:
  /// The output brake force
  real_type mForce;
  /// The frictions state
  real_type mZ;
  /// The frictions stes derivative
  real_type mZDeriv;
  /// The maximum force when brakes are applied
  real_type mMaxForce;
  /// The maximum force when brakes are not applied
  real_type mMinForce;

  /// The intput port which must provide the position
  RealPortHandle mBrakePressurePort;
  /// The intput port which must provide the velocity
  RealPortHandle mVelocityPort;
};

} // namespace OpenFDM

#endif
