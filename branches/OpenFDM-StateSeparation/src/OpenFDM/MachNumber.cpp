/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "MachNumber.h"

#include "TypeInfo.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(MachNumber, Model)
  END_OPENFDM_OBJECT_DEF

MachNumber::MachNumber(const std::string& name) :
  Model(name),
  mVelocityPort(this, "velocity", Size(3, 1), true),
  mSoundSpeedPort(this, "soundSpeed", true),
  mMachNumberPort(this, "machNumber")
{
}

MachNumber::~MachNumber(void)
{
}

void
MachNumber::output(const Task&, const DiscreteStateValueVector&,
                   const ContinousStateValueVector&,
                   PortValueList& portValues) const
{
  //     Vector3 v = Vector6(portValues[mVelocityPort]).getLinear();
  Vector3 v = portValues[mVelocityPort];
  real_type soundSpeed = portValues[mSoundSpeedPort];
  real_type eps = Limits<real_type>::epsilon();
  portValues[mMachNumberPort] = norm(v)/(soundSpeed + eps);
}

} // namespace OpenFDM
