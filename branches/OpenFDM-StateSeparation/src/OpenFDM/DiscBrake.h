/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
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

  virtual void init(const Task&,DiscreteStateValueVector&,
                    ContinousStateValueVector&, const PortValueList&) const;
  virtual void output(const Task&, const DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const;
  virtual void derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector&,
                          const PortValueList&,
                          ContinousStateValueVector&) const;

  const real_type& getMinForce(void) const;
  void setMinForce(const real_type& minForce);

  const real_type& getMaxForce(void) const;
  void setMaxForce(const real_type& maxForce);

  const real_type& getSigma(void) const;
  void setSigma(const real_type& sigma);

private:
  /// The maximum force when brakes are applied
  real_type mMaxForce;
  /// The maximum force when brakes are not applied
  real_type mMinForce;
  /// The stiffness of the friction model
  real_type mSigma;

  // State of this model
  SharedPtr<Vector1StateInfo> mZStateInfo;

  /// The input port which must provide brake input
  RealInputPort mBrakeInputPort;
  /// The input port which must provide the velocity
  RealInputPort mVelocityPort;
  /// The resulting force output
  RealOutputPort mForcePort;
};

} // namespace OpenFDM

#endif
