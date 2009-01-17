/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_PacejkaTire_H
#define OpenFDM_PacejkaTire_H

#include "SingleLinkInteract.h"

namespace OpenFDM {

class PacejkaTire : public SingleLinkInteract {
  OPENFDM_OBJECT(PacejkaTire, SingleLinkInteract);
  class Context;
public:
  PacejkaTire(const std::string& name);
  virtual ~PacejkaTire(void);

  virtual MechanicContext*
  newMechanicContext(const Environment*, PortValueList&) const;

  /** Set wheel axis direction.
   */
  const Vector3& getAxis(void) const;
  void setAxis(const Vector3& axis);

  /** Set wheel radius.
   */
  void setWheelRadius(const real_type& wheelRadius);
  const real_type& getWheelRadius(void) const;

  // Model function to compute the resulting force
  virtual Vector6 getForce(const real_type& rho, const real_type& rhoDot,
                           const real_type& alpha, const real_type& kappa,
                           const real_type& gamma, const real_type& phi) const = 0;

private:
  RealOutputPort mSideSlipPort;
  RealOutputPort mLongitudinalSlipPort;
  RealOutputPort mCamberAnglePort;
  RealOutputPort mNormalForcePort;
  RealOutputPort mLateralForcePort;
  RealOutputPort mLongitudinalForcePort;

  Vector3 mAxis;
  real_type mWheelRadius;
};

} // namespace OpenFDM

#endif
