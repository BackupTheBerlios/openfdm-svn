/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "WindAxis.h"

#include "TypeInfo.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(WindAxis, Model)
  END_OPENFDM_OBJECT_DEF

WindAxis::WindAxis(const std::string& name) :
  Model(name),
  mVelocityPort(this, "bodyVelocity", Size(3, 1), true),
  mAlphaPort(this, "alpha"),
  mAlphaDotPort(this, "alphaDot"),
  mBetaPort(this, "beta"),
  mBetaDotPort(this, "betaDot"),
  mAirSpeedPort(this, "airSpeed")
{
}

WindAxis::~WindAxis(void)
{
}

void
WindAxis::output(const Task&, const DiscreteStateValueVector&,
                 const ContinousStateValueVector&,
                 PortValueList& portValues) const
{
  Vector3 v = portValues[mVelocityPort];
  
  real_type alpha = 0;
  if (Limits<real_type>::min() < fabs(v(0)))
    alpha = atan2(v(2), v(0));
  portValues[mAlphaPort] = alpha;
  
  real_type beta = 0;
  real_type uw = sqrt(v(0)*v(0) + v(2)*v(2));
  if (Limits<real_type>::min() < fabs(uw))
    beta = atan2(v(1), uw);
  portValues[mBetaPort] = beta;
  
  real_type vt = norm(v);
  portValues[mAirSpeedPort] = vt;
  
  // FIXME: we need something that avoids the singularities when just doing
  // finite differences with alpha and beta.
  // may be we can also get then a representation of alpha and beta that is
  // steady at zero velocity
  portValues[mAlphaDotPort] = 0;
  portValues[mBetaDotPort] = 0;
}

} // namespace OpenFDM
