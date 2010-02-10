/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "DynamicPressure.h"

#include "TypeInfo.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(DynamicPressure, Model)
  DEF_OPENFDM_PROPERTY(Vector3, Direction, Serialized)
  END_OPENFDM_OBJECT_DEF

DynamicPressure::DynamicPressure(const std::string& name) :
  Model(name),
  mVelocityPort(this, "velocity", Size(3, 1), true),
  mDensityPort(this, "density", true),
  mDynamicPressurePort(this, "dynamicPressure"),
  mDirection(0, 0, 0)
{
}

DynamicPressure::~DynamicPressure(void)
{
}

void
DynamicPressure::output(const Task&, const DiscreteStateValueVector&,
                        const ContinousStateValueVector&,
                        PortValueList& portValues) const
{
  //     Vector3 v = Vector6(portValues[mVelocityPort]).getLinear();
  Vector3 v = portValues[mVelocityPort];
  real_type density = portValues[mDensityPort];
  if (mDirection == Vector3::zeros())
    portValues[mDynamicPressurePort] = 0.5*density*dot(v, v);
  else {
    real_type dv = dot(mDirection, v);
    real_type dd = dot(mDirection, mDirection);
    portValues[mDynamicPressurePort] = 0.5*density*dv*dv/dd;
  }
}

} // namespace OpenFDM
