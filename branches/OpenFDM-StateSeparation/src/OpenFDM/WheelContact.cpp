/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "WheelContact.h"

#include "LogStream.h"
#include "PortValueList.h"
#include "Task.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(WheelContact, SingleLinkInteract)
  DEF_OPENFDM_PROPERTY(Vector3, Position, Serialized)
  DEF_OPENFDM_PROPERTY(Vector3, Axis, Serialized)
  DEF_OPENFDM_PROPERTY(Real, WheelRadius, Serialized)
  DEF_OPENFDM_PROPERTY(Real, SpringConstant, Serialized)
/// FIXME want to have similar names than with linearspringdamper
  DEF_OPENFDM_PROPERTY(Real, SpringDamping, Serialized)
  DEF_OPENFDM_PROPERTY(Real, FrictionCoeficient, Serialized)
  END_OPENFDM_OBJECT_DEF

WheelContact::WheelContact(const std::string& name) :
  SingleLinkInteract(name),
  mPosition(0, 0, 0),
  mAxis(0, 1, 0)
{
  mWheelRadius = 0.3;
  mSpringConstant = 0;
  mSpringDamping = 0;
  mFrictionCoeficient = 0.8;
}

WheelContact::~WheelContact(void)
{
}

void
WheelContact::articulation(const Task& task, const Environment& environment,
                           const ContinousStateValueVector&,
                           PortValueList& portValues) const
{
  const CoordinateSystem& cs = portValues[mMechanicLink].getCoordinateSystem();

  // FIXME, for now relative position
  Vector3 position = mPosition - portValues[mMechanicLink].getDesignPosition();

  // The coordinate system at the hub.
  CoordinateSystem hubCoordinateSystem(cs.getRelative(position));

  // Get the ground values in the hub coordinate system.
  GroundValues groundValues =
    environment.getGroundPlane(hubCoordinateSystem, task.getTime());

  // Transform the plane equation to the local frame.
  Plane lp = groundValues.plane;
 
  // Get the intersection length.
  real_type distHubGround = fabs(lp.getDist());
  real_type compressLength = mWheelRadius - distHubGround;
  
  // Don't bother if we do not intersect the ground.
  if (compressLength < 0)
    return;

  Vector3 contactPoint = distHubGround*lp.getNormal();
  
  // The velocity of the ground patch in the current frame.
  Vector6 groundVel = groundValues.vel;
  // Now get the relative velocity of the ground wrt the hub
  Vector6 relVel
    = portValues[mMechanicLink].getReferenceVelocity(position) - groundVel;


  // The velocity perpandicular to the plane.
  // Positive when the contact spring is compressed,
  // negative when decompressed.
  real_type compressVel = - lp.scalarProjectToNormal(relVel.getLinear());
  
  // Get a transform from the current frames coordinates into
  // wheel coordinates.
  // The wheel coordinates x axis is defined by the forward orientation
  // of the wheel, the z axis points perpandicular to the ground
  // plane downwards.
  Vector3 forward = normalize(cross(mAxis, lp.getNormal()));
  Vector3 side = normalize(cross(lp.getNormal(), forward));

  // Transformed velocity to the ground plane
  Vector2 wheelVel(dot(forward, relVel.getLinear()),
                   dot(side, relVel.getLinear()));

  // The wheel rotation speed wrt ground
  Vector3 rotVel = relVel.getAngular();
  real_type omegaR = dot(rotVel, mAxis) * distHubGround;

//   Log(Model,Error) << trans(groundVel) << " "
//                    << trans(wheelVel) << " "
//                    << omegaR << " "
//                    << compressLength << " "
//                    << distHubGround << endl;


  // Get the plane normal force.
  real_type normForce = computeNormalForce(compressLength, compressVel);
  // The normal force cannot get negative here.
  normForce = max(static_cast<real_type>(0), normForce);

  // Get the friction force.
  Vector2 fricForce = computeFrictionForce(normForce, wheelVel,
                                           omegaR, groundValues.friction);
  
  // The resulting force is the sum of both.
  // The minus sign is because of the direction of the surface normal.
  Vector3 force = - fricForce(0)*forward - fricForce(1)*side
    - normForce*lp.getNormal();
  
  // We don't have an angular moment.
  portValues[mMechanicLink].applyForce(forceFrom(contactPoint, force));
}

real_type
WheelContact::computeNormalForce(real_type compressLen, real_type compressVel) const
{
  return compressLen*mSpringConstant
    + mSpringDamping*min(compressVel, static_cast<real_type>(0));
}

Vector2
WheelContact::computeFrictionForce(real_type normForce, const Vector2& vel,
                                   real_type omegaR, real_type friction) const
{
  // We just get the wheel slip directly here
  real_type wheelSlip = vel(0)+omegaR;
  
  // The slip angle is the angle between the 'velocity vector' and 
  // the wheel forward direction.
  real_type slipAngle = rad2deg*atan2(vel(1), fabs(vel(0)));
//   slipAngle = saturate(slipAngle, 10*fabs(vel(1)));
  slipAngle = smoothSaturate(slipAngle, 10*fabs(vel(1)));
  
//   Vector2 slip(wheelSlip, slipAngle);
//   if (1 < norm(slip))
//     slip = normalize(slip);
  Vector2 slip(smoothSaturate(wheelSlip, real_type(1)),
               smoothSaturate(slipAngle, real_type(1)));
  
  // The friction force for fast movement.
  return (-friction*mFrictionCoeficient*normForce)*slip;
}

const Vector3&
WheelContact::getPosition(void) const
{
  return mPosition;
}

void
WheelContact::setPosition(const Vector3& position)
{
  mPosition = position;
}

const Vector3&
WheelContact::getAxis(void) const
{
  return mAxis;
}

void
WheelContact::setAxis(const Vector3& axis)
{
  if (norm(axis) <= Limits<real_type>::safe_min())
    return;
  mAxis = normalize(axis);
}

} // namespace OpenFDM
