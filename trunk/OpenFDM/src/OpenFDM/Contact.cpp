/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Contact.h"

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Contact, ExternalForce)
  END_OPENFDM_OBJECT_DEF

Contact::Contact(const std::string& name)
  : ExternalForce(name)
{
  setDisableMode(Model::ResetHold);

  // FIXME??
  addSampleTime(SampleTime::PerTimestep);
  addSampleTime(SampleTime::Continous);
}

Contact::~Contact(void)
{
}

bool
Contact::init(void)
{
  setForce(Vector6::zeros());
  return ExternalForce::init();
}

void
Contact::output(const TaskInfo& taskInfo)
{
  if (nonZeroIntersection(taskInfo.getSampleTimeSet(),
                          SampleTime::PerTimestep)) {
    Log(Model, Debug) << "Contact::output(): \"" << getName()
                      << "\" computing ground plane below" << endl;
    getGround(taskInfo.getTime());
  }

  // Transform the plane equation to the local frame.
  Plane lp = mMountFrame->planeFromRef(mGroundVal.plane);
  
  // Get the intersection length.
  real_type compressLength = lp.getDist(Vector3::zeros());
  
  // Don't bother if we do not intersect the ground.
  if (compressLength < 0) {
    setForce(Vector6::zeros());
    return;
  }
  
  // The velocity of the ground patch in the current frame.
  Vector6 groundVel(mMountFrame->rotFromRef(mGroundVal.vel.getAngular()),
                    mMountFrame->rotFromRef(mGroundVal.vel.getLinear()));
  groundVel -= mMountFrame->getRefVel();
  // Now get the relative velocity of the ground wrt the contact point
  Vector3 relVel = - groundVel.getLinear();

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
  setForce(Vector6(Vector3::zeros(), force));
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
Contact::setEnvironment(Environment* environment)
{
  mEnvironment = environment;
}

void
Contact::getGround(real_type t)
{
  // Get the position of the contact in the reference system.
  Vector3 pos = mMountFrame->getRefPosition();
  // Query for the ground parameters at this point.
  mGroundVal = mEnvironment->getGround()->getGroundPlane(t, pos);
}

} // namespace OpenFDM
