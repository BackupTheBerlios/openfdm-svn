/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"
#include "Tailhook.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Tailhook, ExternalForce)
  DEF_OPENFDM_PROPERTY(Real, Length, Serialized)
  DEF_OPENFDM_PROPERTY(Real, UpAngle, Serialized)
  DEF_OPENFDM_PROPERTY(Real, DownAngle, Serialized)
  END_OPENFDM_OBJECT_DEF

Tailhook::Tailhook(const std::string& name) :
  ExternalForce(name),
  mLength(0.6),
  mUpAngle(0.5),
  mDownAngle(-0.5),
  mAngularVelocity(1)
{
  mWireFrame = new FreeFrame("Wire frame");
  mMountFrame->addChildFrame(mWireFrame);

  setNumDiscreteStates(1);

  // FIXME??
  addSampleTime(SampleTime::PerTimestep);
  addSampleTime(SampleTime::Continous);

  setNumInputPorts(1);
  setInputPortName(0, "hookPosition");

  setNumOutputPorts(1);
  setOutputPort(0, "angle", this, &Tailhook::getAngle);
}

Tailhook::~Tailhook(void)
{
}

bool
Tailhook::init(void)
{
  mHasWire = false;
  mFirstTime = true;
  mAngleCommand = mUpAngle;

  mHookPositionPort = getInputPort(0)->toRealPortHandle();
  if (!mHookPositionPort.isConnected()) {
    Log(Model, Error) << "Initialization of Tailhook model \"" << getName()
                      << "\" failed: Input port \"" << getInputPortName(0)
                      << "\" is not connected!" << endl;
    return false;
  }

  return ExternalForce::init();
}

void
Tailhook::output(const TaskInfo& taskInfo)
{
  if (nonZeroIntersection(taskInfo.getSampleTimeSet(),
                          SampleTime::PerTimestep)) {
    Log(Model, Debug) << "Tailhook::output(): \"" << getName()
                      << "\" computing ground plane below" << endl;
    getGround(taskInfo.getTime());
  }

  // The tailhook angle corrected for no ground intersection
  mAngle = computeCurrentTailhookAngle();

  if (!mHasWire || mFirstTime) {
    setForce(Vector6::zeros());
    return;
  }

  // Query the wire, write the result into the attached frame
  // ... yes this function works through sideffects ... :-/
  real_type width;
  if (!computeWireFrame(taskInfo.getTime(), width)) {
    mHasWire = false;
    setForce(Vector6::zeros());
    return;
  }

  // The wire endpoints
  Vector3 wireEnd0 = mWireFrame->posToParent(width*0.5*Vector3::unit(2));
  Vector3 wireEnd1 = mWireFrame->posToParent(-width*0.5*Vector3::unit(2));

//   Log(Model,Error) << trans(wireEnd0) << trans(wireEnd1) << endl;

  // The intersection of the x/z plane with the line between the wire ends
  Vector3 wireDir = wireEnd1 - wireEnd0;
  Vector3 hookWireInters = (1/width*wireDir(2))*wireDir + wireEnd0;
  // Ok, the hook intersects the wire but the aircraft is sufficiently
  // far that the hook tip has reached the wire
  if (norm(hookWireInters) < mLength) {
    setForce(Vector6::zeros());
    return;
  }

  // from the hooks mount together with the wire endpoints we get a plane
  // The plane normal is:
  Vector3 normal = normalize(cross(wireEnd0, wireEnd1));
  // make the normal point downwards

  // now determine the hooks pos position as it lies axactly in this plane
  mAngle = atan(-normal(1)/normal(3));

  // now the relative velocities
  Vector3 linVel = cross(mWireFrame->getRelVel().getAngular(), Vector3::unit(2));
  Vector6 relVel0 = mWireFrame->motionToParent(mWireFrame->getRelVel() + Vector6(Vector3::zeros(), width*0.5*linVel));
  Vector6 relVel1 = mWireFrame->motionToParent(mWireFrame->getRelVel() - Vector6(Vector3::zeros(), width*0.5*linVel));

//   Vector3 tipPos(-mLength*cos(mAngle), 0, -mLength*sin(mAngle));

  Vector3 wireDir0 = normalize(wireEnd0);
  Vector3 wireDir1 = normalize(wireEnd1);

  real_type vel0 = dot(relVel0.getLinear(), wireDir0);
  real_type vel1 = dot(relVel0.getLinear(), wireDir1);

// Log(Model,Error) << vel0 << "  " << vel1 << endl;
  
  real_type v = 0.5*(vel0 + vel1);
  if (v < 0.1) {
    mHasWire = false;
    setForce(Vector6::zeros());
    return;
  }

  setForce(Vector6(Vector3::zeros(), (5e4 + v*3e3)*(wireDir0 + wireDir1)));
}

void
Tailhook::update(const TaskInfo& taskInfo)
{
  /// The current hook tip's position
  Vector3 tipPos(-cos(mAngle)*mLength, 0, -sin(mAngle)*mLength);

  /// The current hooks position and time
  HookPosition currentPosition;
  currentPosition.t = taskInfo.getTime();
  currentPosition.basePosition = mMountFrame->getRefPosition();
  currentPosition.hookVector = mMountFrame->rotToRef(tipPos);

  if (!mFirstTime && !mHasWire) {
    const Ground* ground = mEnvironment->getGround();
    mHasWire = ground->caughtWire(mOldHookPosition, currentPosition);
    if (mHasWire)
      Log(Model,Debug) << "Caught wire!" << endl;
  }
  mOldHookPosition = currentPosition;
  mFirstTime = false;

  real_type hookCommand = mHookPositionPort.getRealValue();
  hookCommand = hookCommand*mDownAngle + (1-hookCommand)*mUpAngle;
  real_type angleError = hookCommand - mAngleCommand;
  angleError = sign(angleError)*min(mAngularVelocity, 40*fabs(angleError));
  /// FIXME: isPerTimestep sample times do not contain the step size ...
  /// hardwire that ATM
  mAngleCommand += 1/120.0*angleError;
}

void
Tailhook::setDiscreteState(const StateStream& state)
{
  state.readSubState(mAngleCommand);
}

void
Tailhook::getDiscreteState(StateStream& state) const
{
  state.writeSubState(mAngleCommand);
}

void
Tailhook::setEnvironment(Environment* environment)
{
  mEnvironment = environment;
}

void
Tailhook::getGround(real_type t)
{
  // Get the position of the contact in the reference system.
  Vector3 pos = mMountFrame->getRefPosition();
  // Query for the ground parameters at this point.
  mGroundVal = mEnvironment->getGround()->getGroundPlane(t, pos);
}

bool
Tailhook::computeWireFrame(real_type t, real_type& width)
{
  WireValues wireVals;
  const Ground* ground = mEnvironment->getGround();
  // Early return if no cat in range
  if (!ground->getWireEnds(t, wireVals)) {
    mHasWire = false;
    return false;
  }

  mWireFrame->setRefPosition(wireVals.position);
  mWireFrame->setRefOrientation(wireVals.orientation);
  Vector6 locVel(mWireFrame->rotFromRef(wireVals.velocity.getAngular()),
                 mWireFrame->rotFromRef(wireVals.velocity.getLinear()));
  mWireFrame->setRefVel(locVel);
  width = wireVals.width;

  return true;
}

real_type
Tailhook::computeCurrentTailhookAngle(void)
{
  // Transform the plane equation to the local frame.
  Plane lp = mMountFrame->planeFromRef(mGroundVal.plane);

  // Now compute the intersection of the circle where the tip can move
  // with the ground plane. If there is no intersection movement is free

  // The trick is to find a xz = (x, 0, z) with |xz| = r and dot(xz, n) + d = 0
  // where n is the plane normal and d is the plane distance

  Vector n = lp.getNormal();
  real_type d = lp.getDist();

  // we are paralell to the plane
  if (fabs(n(3)) <= Limits<real_type>::min())
    return mAngleCommand;

  // that leads to a quadratic equation where we pick the solution pointing
  // backwards:
  real_type nx2nz2 = n(1)*n(1) + n(3)*n(3);

  // we need to didive through that later, with exact operations it should
  // be safe to not check that because of n(3) being bounded away from zero,
  // but due to the square the value can underflow
  if (fabs(nx2nz2) <= Limits<real_type>::min())
    return mAngleCommand;

  // the discriminant (rought german translation ...)
  real_type discr = nx2nz2*mLength*mLength - d*d;
  // Unconstraint angle position
  if (discr <= 0)
    return mAngleCommand;

  // the x coorinate of the tip
  real_type x = -(d*n(1) + fabs(n(3))*sqrt(discr))/nx2nz2;

  // get the z coordinate of the tip from the plane equation
  real_type z = -(d + x*n(1))/n(3);

  // ok, now the angle ...
  real_type angle = atan2(-z, -x);
  // limit to the range of movement
  if (angle < mAngleCommand)
    angle = mAngleCommand;
  if (mUpAngle < angle)
    angle = mUpAngle;

  return angle;
}

} // namespace OpenFDM
