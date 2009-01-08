/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "SimpleGear.h"

#include "Assert.h"
#include "LogStream.h"
#include "Unit.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"
#include "Contact.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(SimpleGear, Contact)
  DEF_OPENFDM_PROPERTY(Real, SpringConstant, Serialized)
/// FIXME want to have similar names than with linearspringdamper
  DEF_OPENFDM_PROPERTY(Real, SpringDamping, Serialized)
  DEF_OPENFDM_PROPERTY(Real, FrictionCoeficient, Serialized)

  DEF_OPENFDM_PROPERTY(Real, SteeringAngle, NotSerialized)
/// FIXME think about that name 
  DEF_OPENFDM_PROPERTY(Real, Brake, NotSerialized)
  END_OPENFDM_OBJECT_DEF

SimpleGear::SimpleGear(const std::string& name)
  : Contact(name)
{
  mSteeringAngle = 0;
  mBrake = 0;
  mSpringConst = 0;
  mSpringDamp = 0;
  mFrictionCoef = 0;

  /// FIXME
  unsigned inputPortBase = getNumInputPorts();
  setNumInputPorts(inputPortBase + 2);
  setInputPortName(inputPortBase + 0, "brakeCommand");
  setInputPortName(inputPortBase + 1, "steeringAngle");
}

SimpleGear::~SimpleGear(void)
{
}

bool
SimpleGear::init(void)
{
  NumericPortAcceptor* port = getInputPort("brakeCommand");
  if (port)
    mBrakeCommandHandle = port->toRealPortHandle();
  else
    mBrakeCommandHandle = RealPortHandle(0);

  port = getInputPort("steeringAngle");
  if (port)
    mSteeringAngleHandle = port->toRealPortHandle();
  else
    mSteeringAngleHandle = RealPortHandle(0);

  mBrake = 0;
  mSteeringAngle = 0;

  return Contact::init();
}

void
SimpleGear::output(const TaskInfo& taskInfo)
{
  if (nonZeroIntersection(taskInfo.getSampleTimeSet(),
                          SampleTime::PerTimestep)) {
    if (mBrakeCommandHandle.isConnected())
      mBrake = mBrakeCommandHandle.getRealValue();
    if (mSteeringAngleHandle.isConnected())
      mSteeringAngle = mSteeringAngleHandle.getRealValue();
  }

  Contact::output(taskInfo);
}

real_type
SimpleGear::getSteeringAngle(void) const
{
  return mSteeringAngle;
}

void
SimpleGear::setSteeringAngle(real_type steeringAngle)
{
  mSteeringAngle = steeringAngle;
}

real_type
SimpleGear::getBrake(void) const
{
  return mBrake;
}

void
SimpleGear::setBrake(real_type brake)
{
  mBrake = brake;
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
SimpleGear::getSpringDamping(void) const
{
  return mSpringDamp;
}

void
SimpleGear::setSpringDamping(real_type springDamp)
{
  mSpringDamp = springDamp;
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
                               real_type compressVel) const
{
  return compressLen*mSpringConst
    - mSpringDamp*min(compressVel, static_cast<real_type>(0));
}

// Compute the friction force.
Vector3
SimpleGear::computeFrictionForce(real_type normForce, const Vector3& vel,
                                 const Vector3& groundNormal,
                                 real_type friction) const
{
  // Get a transform from the current frames coordinates into
  // wheel coordinates.
  // The wheel coordinates x asxis is defined by the forward orientation
  // of the wheel, the z axis points perpandicular to the ground
  // plane downwards.
  Vector3 forward(cos(mSteeringAngle), sin(mSteeringAngle), 0);
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
  real_type wheelSlip = interpolate(mBrake,
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
