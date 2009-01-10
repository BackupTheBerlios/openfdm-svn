/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "SimpleGear.h"

#include "Assert.h"
#include "LogStream.h"
#include "Unit.h"
#include "Object.h"
#include "Vector.h"
#include "Contact.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(SimpleGear, Contact)
  DEF_OPENFDM_PROPERTY(Bool, EnableSteeringAngle, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableBrakeCommand, Serialized)
  DEF_OPENFDM_PROPERTY(Real, SpringConstant, Serialized)
  DEF_OPENFDM_PROPERTY(Real, DamperConstant, Serialized)
  DEF_OPENFDM_PROPERTY(Real, FrictionCoeficient, Serialized)
  END_OPENFDM_OBJECT_DEF

SimpleGear::SimpleGear(const std::string& name) :
  Contact(name),
  mSpringConst(0),
  mDamperConst(0),
  mFrictionCoef(0)
{
}

SimpleGear::~SimpleGear(void)
{
}

void
SimpleGear::setEnableSteeringAngle(bool enable)
{
  if (enable == getEnableSteeringAngle())
    return;
  if (enable)
    mSteeringAnglePort = RealInputPort(this, "steeringAngle", true);
  else
    mSteeringAnglePort.clear();
}

bool
SimpleGear::getEnableSteeringAngle() const
{
  return !mSteeringAnglePort.empty();
}

void
SimpleGear::setEnableBrakeCommand(bool enable)
{
  if (enable == getEnableBrakeCommand())
    return;
  if (enable)
    mBrakeCommandPort = RealInputPort(this, "brakeCommand", true);
  else
    mBrakeCommandPort.clear();
}

bool
SimpleGear::getEnableBrakeCommand() const
{
  return !mBrakeCommandPort.empty();
}

real_type
SimpleGear::getSpringConstant(void) const
{
  return mSpringConst;
}

void
SimpleGear::setSpringConstant(real_type springConst)
{
  mSpringConst = springConst;
}

real_type
SimpleGear::getDamperConstant(void) const
{
  return mDamperConst;
}

void
SimpleGear::setDamperConstant(real_type damperConst)
{
  mDamperConst = damperConst;
}

real_type
SimpleGear::getFrictionCoeficient(void) const
{
  return mFrictionCoef;
}

void
SimpleGear::setFrictionCoeficient(real_type frictionCoef)
{
  mFrictionCoef = frictionCoef;
}

// Compute the plane normal force.
real_type
SimpleGear::computeNormalForce(real_type compressLen,
                               real_type compressVel,
                               PortValueList& portValueList) const
{
  return compressLen*mSpringConst - mDamperConst*compressVel;
}

// Compute the friction force.
Vector3
SimpleGear::computeFrictionForce(real_type normForce, const Vector3& vel,
                                 const Vector3& groundNormal,
                                 real_type friction,
                                 PortValueList& portValueList) const
{
  // Get the relevant inputs or their defaults.
  real_type steeringAngle = 0;
  if (getEnableSteeringAngle())
    steeringAngle = portValueList[mSteeringAnglePort];
  real_type brakeCommand = 0;
  if (getEnableBrakeCommand())
    brakeCommand = portValueList[mBrakeCommandPort];

  // Get a transform from the current frames coordinates into
  // wheel coordinates.
  // The wheel coordinates x asxis is defined by the forward orientation
  // of the wheel, the z axis points perpandicular to the ground
  // plane downwards.
  Vector3 forward(cos(steeringAngle), sin(steeringAngle), 0);
  Vector3 side = cross(groundNormal, forward);
  forward = normalize(cross(side, groundNormal));
  side = normalize(side);

  // Transformed to the ground plane
  Vector2 wheelVel(dot(forward, vel), dot(side, vel));

  // Now we compute something like the JSBSim gear friction model.
  // The x coordinate is in wheel forward direction,
  // the y coordinate points towards right. 
  
  // The wheel spin speed is not known in this simple model.
  // We just set that to 0.99999 times the x-velocity in the 
  // rolling case and to 1 in the brakeing case.
  real_type wheelSlip = interpolate(brakeCommand,
                                    (real_type)0, 1e-5*wheelVel(0),
                                    (real_type)1, wheelVel(0));
  
  // The slip angle is the angle between the 'velocity vector' and 
  // the wheel forward direction.
  real_type slipAngle = rad2deg*atan2(wheelVel(1), fabs(wheelVel(0)));
//   slipAngle = saturate(slipAngle, 10*fabs(wheelVel(2)));
  slipAngle = smoothSaturate(slipAngle, 10*fabs(wheelVel(1)));
  
//   Vector2 slip(wheelSlip, slipAngle);
//   if (1 < norm(slip))
//     slip = normalize(slip);
  Vector2 slip(smoothSaturate(wheelSlip, real_type(1)),
               smoothSaturate(slipAngle, real_type(1)));
  
  // The friction force for fast movement.
  Vector2 fricForce = (-friction*mFrictionCoef*normForce)*slip;
  
  // Transform the friction force back and return
  return fricForce(0)*forward + fricForce(1)*side;
}

} // namespace OpenFDM
