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
#include "Contact.h"

namespace OpenFDM {

Contact::Contact(const std::string& name, Environment* env)
  : ExternalForce(name)
{
  mEnabled = true;
  mEnvironment = env;
  setPosition(Vector3::zeros());

  unsigned inputPortBase = getNumInputPorts();
  setNumInputPorts(inputPortBase + 1);
  setInputPortName(inputPortBase + 0, "enabled");

  // FIXME??
  addSampleTime(SampleTime::PerTimestep);
  addSampleTime(SampleTime::Continous);
}

Contact::~Contact(void)
{
}

void
Contact::output(const TaskInfo& taskInfo)
{
  if (nonZeroIntersection(taskInfo.getSampleTimeSet(),
                          SampleTime::PerTimestep)) {
    Log(Model, Debug) << "Contact::output(): \"" << getName()
                      << "\" computing ground plane below" << endl;
    getGround(taskInfo.getTime());

    // FIXME
    if (getInputPort("enabled").isValid())
      mEnabled = 0.5 < getInputPort("enabled").getValue().toReal();
  }
}

void
Contact::computeForce(void)
{
  const Frame* frame = getParentFrame(0);
  OpenFDMAssert(frame);
  if (!frame || !mEnabled) {
    applyForce(Vector6::zeros());
    return;
  }

  // Transform the plane equation to the local frame.
  Plane lp = frame->planeFromRef(mGroundVal.plane);
  
  // Get the intersection length.
  real_type compressLength = lp.getDist(getPosition());
  
  // Don't bother if we do not intersect the ground.
  if (compressLength < 0) {
    applyForce(Vector6::zeros());
    return;
  }
  
  // The velocity of the ground patch in the current frame.
  Vector6 groundVel = frame->motionFromRef(mGroundVal.vel);
  // Now get the relative velocity of the ground wrt the contact point
  Vector3 relVel = - motionTo(getPosition(), groundVel).getLinear();

  // The velocity perpandicular to the plane.
  // Positive when the contact spring is compressed,
  // negative when decompressed.
  real_type compressVel = - lp.scalarProjectToNormal(relVel);
  
  // The in plane velocity.
  Vector3 sVel = lp.projectToPlane(relVel);
  
  // Get the plane normal force.
  real_type normForce = computeNormalForce(compressLength, compressVel);
  // The normal force cannot get negative here.
  normForce = max(static_cast<real_type>(0), normForce);
  
  // Get the friction force.
  Vector3 fricForce = computeFrictionForce(normForce, sVel, lp.getNormal(),
                                           mGroundVal.friction);
  
  // The resulting force is the sum of both.
  // The minus sign is because of the direction of the surface normal.
  Vector3 force = fricForce - normForce*lp.getNormal();
  
  // We don't have an angular moment.
  applyForce(forceFrom(getPosition(), force));
}

real_type
Contact::computeNormalForce(real_type compressLen, real_type compressVel) const
{
  return 0;
}

Vector3
Contact::computeFrictionForce(real_type normForce, const Vector3& vel,
                              const Vector3& groundNormal,
                              real_type friction) const
{
  return Vector3::zeros();
}

void
Contact::getGround(real_type t)
{
  const Frame* frame = getParentFrame(0);
  OpenFDMAssert(frame);
  if (!frame)
    return;

  // Get the position of the contact in the reference system.
  Vector3 pos = frame->posToRef(getPosition());
  // Query for the ground parameters at this point.
  mGroundVal = mEnvironment->getGround()->getGroundPlane(t, pos);
}

} // namespace OpenFDM
