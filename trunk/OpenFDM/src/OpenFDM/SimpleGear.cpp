/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "LogStream.h"
#include "Units.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"
#include "Contact.h"
#include "SimpleGear.h"

namespace OpenFDM {

SimpleGear::SimpleGear(const std::string& name)
  : Contact(name)
{
  mSteeringAngle = 0;
  mBrake = 0;
  mSpringConst = 0;
  mSpringDamp = 0;
  mFrictionCoef = 0;

  addProperty("steeringAngle",
              Property(this, &SimpleGear::getSteeringAngle,
                       &SimpleGear::setSteeringAngle));
  addProperty("brake",
              Property(this, &SimpleGear::getBrake, &SimpleGear::setBrake));
  addProperty("springConstant",
              Property(this, &SimpleGear::getSpringConstant,
                       &SimpleGear::setSpringConstant));
  addProperty("springDamping",
              Property(this, &SimpleGear::getSpringDamping,
                       &SimpleGear::setSpringDamping));
  addProperty("frictionCoeficient",
              Property(this, &SimpleGear::getFrictionCoeficient,
                       &SimpleGear::setFrictionCoeficient));

  unsigned inputPortBase = getNumInputPorts();
  setNumInputPorts(inputPortBase + 2);
  setInputPortName(inputPortBase + 0, "brakeCommand");
  setInputPortName(inputPortBase + 1, "steeringAngle");
}

SimpleGear::~SimpleGear(void)
{
}

// bool
// SimpleGear::init(void)
// {
//   return true;
// }

void
SimpleGear::output(const TaskInfo& taskInfo)
{
  if (nonZeroIntersection(taskInfo.getSampleTimeSet(),
                          SampleTime::PerTimestep)) {
    if (getInputPort("brakeCommand")->isConnected()) {
      RealPortHandle rh = getInputPort("brakeCommand")->toRealPortHandle();
      mBrake = rh.getRealValue();
    }
    if (getInputPort("steeringAngle")->isConnected()) {
      RealPortHandle rh = getInputPort("steeringAngle")->toRealPortHandle();
      mSteeringAngle = rh.getRealValue();
    }
  }

  Contact::output(taskInfo);
}

real_type
SimpleGear::getSteeringAngle(void) const
{
  return mSteeringAngle;
}

void
SimpleGear::setSteeringAngle(const real_type& steeringAngle)
{
  mSteeringAngle = steeringAngle;
}

real_type
SimpleGear::getBrake(void) const
{
  return mBrake;
}

void
SimpleGear::setBrake(const real_type& brake)
{
  mBrake = brake;
}

real_type
SimpleGear::getSpringConstant(void) const
{
  return mSpringConst;
}

void
SimpleGear::setSpringConstant(const real_type& springConst)
{
  mSpringConst = springConst;
}

real_type
SimpleGear::getSpringDamping(void) const
{
  return mSpringDamp;
}

void
SimpleGear::setSpringDamping(const real_type& springDamp)
{
  mSpringDamp = springDamp;
}

real_type
SimpleGear::getFrictionCoeficient(void) const
{
  return mFrictionCoef;
}

void
SimpleGear::setFrictionCoeficient(const real_type& frictionCoef)
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
                                    (real_type)0, 1e-5*wheelVel(1),
                                    (real_type)1, wheelVel(1));
  
  // The slip angle is the angle between the 'velocity vector' and 
  // the wheel forward direction.
  real_type slipAngle = rad2deg*atan2(wheelVel(2), fabs(wheelVel(1)));
  if (fabs(wheelVel(2)) < fabs(slipAngle))
    slipAngle = wheelVel(2);
  
  Vector2 slip(wheelSlip, slipAngle);
//   if (1 < norm(slip))
//     slip = normalize(slip);
  if (1 < fabs(wheelSlip))
    slip(1) = sign(wheelSlip);
  if (1 < fabs(slipAngle))
    slip(2) = sign(slipAngle);
  
  // The friction force for fast movement.
  Vector2 fricForce = (-friction*mFrictionCoef*normForce)*slip;
  
  // Transform the friction force back and return
  return fricForce(1)*forward + fricForce(2)*side;
}

} // namespace OpenFDM
