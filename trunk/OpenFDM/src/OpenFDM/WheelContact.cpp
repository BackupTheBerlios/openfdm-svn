/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"
#include "Environment.h"
#include "WheelContact.h"

namespace OpenFDM {

WheelContact::WheelContact(const std::string& name)
  : ExternalForce(name)
{
  mWheelRadius = 0.3;
  mSpringConstant = 0;
  mSpringDamping = 0;
  mFrictionCoeficient = 0.8;

  // FIXME??
  addSampleTime(SampleTime::PerTimestep);
  addSampleTime(SampleTime::Continous);
}

WheelContact::~WheelContact(void)
{
}

bool
WheelContact::init(void)
{
  mEnvironment = getEnvironment();
  if (!mEnvironment)
    return false;
  return ExternalForce::init();
}

void
WheelContact::output(const TaskInfo& taskInfo)
{
  if (nonZeroIntersection(taskInfo.getSampleTimeSet(),
                          SampleTime::PerTimestep)) {
    Log(Model, Debug) << "WheelContact::output(): \"" << getName()
                      << "\" computing ground plane below" << endl;
    getGround(taskInfo.getTime());
  }
}

void
WheelContact::computeForce(void)
{
  // FIXME: check all signs here, I get the feeling that we have
  // just even much minuses here instead of exactly the right ones ...
  const RigidBody* body = getParentRigidBody(0);
  OpenFDMAssert(body);
  if (!body) {
    applyForce(Vector6::zeros());
    return;
  }
  const Frame* frame = body->getFrame();
  OpenFDMAssert(frame);
  if (!frame) {
    applyForce(Vector6::zeros());
    return;
  }

  // Transform the plane equation to the local frame.
  Plane lp = frame->planeFromRef(mGroundVal.plane);
  
  // Get the intersection length.
  real_type distHubGround = fabs(lp.getDist(Vector3::zeros()));
  real_type compressLength = mWheelRadius - distHubGround;
  
  // Don't bother if we do not intersect the ground.
  if (compressLength < 0) {
    applyForce(Vector6::zeros());
    return;
  }

  Vector3 contactPoint = distHubGround*lp.getNormal();
  
  // The velocity of the ground patch in the current frame.
  Vector6 groundVel(frame->rotFromRef(mGroundVal.vel.getAngular()),
                    frame->rotFromRef(mGroundVal.vel.getLinear()));
  groundVel -= frame->getRefVel();
  // Now get the relative velocity of the ground wrt the hub
  Vector6 relVel = - groundVel;
//   Log(Model,Error) << trans(frame->getRelVel()) << " "
//                    << trans(groundVel) << " "
//                    << trans(frame->motionToParent(relVel)) << endl;


  // The velocity perpandicular to the plane.
  // Positive when the contact spring is compressed,
  // negative when decompressed.
  real_type compressVel = - lp.scalarProjectToNormal(relVel.getLinear());
  
  // Get the plane normal force.
  real_type normForce = computeNormalForce(compressLength, compressVel);
  // The normal force cannot get negative here.
  normForce = max(static_cast<real_type>(0), normForce);
  
  // Get a transform from the current frames coordinates into
  // wheel coordinates.
  // The wheel coordinates x asxis is defined by the forward orientation
  // of the wheel, the z axis points perpandicular to the ground
  // plane downwards.
  Vector3 forward = normalize(cross(Vector3::unit(2), lp.getNormal()));
  Vector3 side = normalize(cross(lp.getNormal(), forward));

  // Transformed velocity to the ground plane
  Vector2 wheelVel(dot(forward, relVel.getLinear()),
                   dot(side, relVel.getLinear()));

  // The wheel rotation speed wrt ground
  Vector3 rotVel = relVel.getAngular();
  real_type omegaR = rotVel(2) * distHubGround;

//   Log(Model,Error) << trans(groundVel) << " "
//                    << trans(wheelVel) << " "
//                    << omegaR << " "
//                    << compressLength << " "
//                    << distHubGround << endl;



  // Get the friction force.
  Vector2 fricForce = computeFrictionForce(normForce, wheelVel,
                                           omegaR, mGroundVal.friction);
  
  // The resulting force is the sum of both.
  // The minus sign is because of the direction of the surface normal.
  Vector3 force = fricForce(1)*forward + fricForce(2)*side
    - normForce*lp.getNormal();
  
  // We don't have an angular moment.
  applyForce(forceFrom(contactPoint, force));
}

real_type
WheelContact::computeNormalForce(real_type compressLen, real_type compressVel) const
{
  return compressLen*mSpringConstant
    - mSpringDamping*min(compressVel, static_cast<real_type>(0));
}

Vector2
WheelContact::computeFrictionForce(real_type normForce, const Vector2& vel,
                                   real_type omegaR, real_type friction) const
{
  // We just get the wheel slip directly here
  real_type wheelSlip = vel(1)+omegaR;
  
  // The slip angle is the angle between the 'velocity vector' and 
  // the wheel forward direction.
  real_type slipAngle = rad2deg*atan2(vel(2), fabs(vel(1)));
//   slipAngle = saturate(slipAngle, 10*fabs(vel(2)));
  slipAngle = smoothSaturate(slipAngle, 10*fabs(vel(2)));
  
//   Vector2 slip(wheelSlip, slipAngle);
//   if (1 < norm(slip))
//     slip = normalize(slip);
  Vector2 slip(smoothSaturate(wheelSlip, 1.0), smoothSaturate(slipAngle, 1.0));
  
  // The friction force for fast movement.
  return (-friction*mFrictionCoeficient*normForce)*slip;
}

void
WheelContact::getGround(real_type t)
{
  const RigidBody* body = getParentRigidBody(0);
  OpenFDMAssert(body);
  if (!body)
    return;
  const Frame* frame = body->getFrame();
  OpenFDMAssert(frame);
  if (!frame)
    return;

  // FIXME
  if (!mEnvironment) {
    mEnvironment = getEnvironment();
  }

  // Get the position of the contact in the reference system.
  Vector3 pos = frame->posToRef(Vector3::zeros());
  // Query for the ground parameters at this point.
  mGroundVal = mEnvironment->getGround()->getGroundPlane(t, pos);
}

} // namespace OpenFDM
