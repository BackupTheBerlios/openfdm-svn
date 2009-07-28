/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "WindAxisForce.h"

#include "TypeInfo.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(WindAxisForce, Model)
  END_OPENFDM_OBJECT_DEF

WindAxisForce::WindAxisForce(const std::string& name) :
  Model(name),
  mAlphaPort(this, "alpha", true),
  mBetaPort(this, "beta", true),
  mDragPort(this, "drag", true),
  mSidePort(this, "side", true),
  mLiftPort(this, "lift", true),
  mForcePort(this, "bodyForce", Size(3, 1))
{
}

WindAxisForce::~WindAxisForce(void)
{
}

void
WindAxisForce::output(const Task&, const DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const
{
  // Get the matrix transforming from stability to body axis
  real_type alpha = portValues[mAlphaPort];
  real_type beta = portValues[mBetaPort];
  real_type ca = cos(alpha);
  real_type sa = sin(alpha);
  real_type cb = cos(beta);
  real_type sb = sin(beta);
  
  // The transform from stability axis to body axis where the
  // stability axis x axis points into the wind, the y axis points
  // to the right and the z axis points downwards
  Matrix33 Tw2b(ca*cb, -ca*sb, -sa,
                   sb,     cb,   0,
                sa*cb, -sa*sb,  ca);
  
  // Get the forces
  real_type drag = 0;
  if (!mDragPort.empty())
    drag = portValues[mDragPort];
  real_type side = 0;
  if (!mSidePort.empty())
    side = portValues[mSidePort];
  real_type lift = 0;
  if (!mLiftPort.empty())
    lift = portValues[mLiftPort];
  
  // Compute the force vector, note that drag is a force pointing
  // afterwards and the lift upwards
  portValues[mForcePort] = Tw2b*Vector3(-drag, side, -lift);
}

} // namespace OpenFDM
