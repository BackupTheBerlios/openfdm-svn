/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "Launchbar.h"

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Force.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Launchbar, ExternalForce)
  DEF_OPENFDM_PROPERTY(Real, Length, Serialized)
  DEF_OPENFDM_PROPERTY(Real, HoldbackLength, Serialized)
  DEF_OPENFDM_PROPERTY(Vector3, HoldbackMount, Serialized)
  DEF_OPENFDM_PROPERTY(Real, UpAngle, Serialized)
  DEF_OPENFDM_PROPERTY(Real, DownAngle, Serialized)
  DEF_OPENFDM_PROPERTY(Real, LaunchForce, Serialized)
  END_OPENFDM_OBJECT_DEF

Launchbar::Launchbar(const std::string& name) :
  ExternalForce(name),
  mLength(0.6),
  mHoldbackLength(1.5),
  mHoldbackMount(0, 0, 0.1),
  mUpAngle(0.5),
  mDownAngle(-0.5),
  mAngularVelocity(1),
  mLaunchForce(0)
{
  mCatFrame = new FreeFrame("Catapult frame");
  mMountFrame->addChildFrame(mCatFrame);

  setNumDiscreteStates(1);

  // FIXME??
  addSampleTime(SampleTime::PerTimestep);
  addSampleTime(SampleTime::Continous);

  setNumInputPorts(2);
  setInputPortName(0, "tryMount");
  setInputPortName(1, "launchCommand");

  setNumOutputPorts(1);
  setOutputPort(0, "angle", this, &Launchbar::getAngle);
}

Launchbar::~Launchbar(void)
{
}

bool
Launchbar::init(void)
{
  mState = Unmounted;
  mAngleCommand = mUpAngle;

  mTryMountPort = getInputPort(0)->toRealPortHandle();
  if (!mTryMountPort.isConnected()) {
    Log(Model, Error) << "Initialization of Launchbar model \"" << getName()
                      << "\" failed: Input port \"" << getInputPortName(0)
                      << "\" is not connected!" << std::endl;
    return false;
  }

  mLaunchCommandPort = getInputPort(1)->toRealPortHandle();
  if (!mLaunchCommandPort.isConnected()) {
    Log(Model, Error) << "Initialization of Launchbar model \"" << getName()
                      << "\" failed: Input port \"" << getInputPortName(1)
                      << "\" is not connected!" << std::endl;
    return false;
  }

  return ExternalForce::init();
}

void
Launchbar::output(const TaskInfo& taskInfo)
{
  if (nonZeroIntersection(taskInfo.getSampleTimeSet(),
                          SampleTime::PerTimestep)) {
    Log(Model, Debug) << "Launchbar::output(): \"" << getName()
                      << "\" computing ground plane below" << std::endl;
    getGround(taskInfo.getTime());
  }

  // The launchbar angle
  mAngle = computeCurrentLaunchbarAngle();

  // Ok, apply the tension at the launchbar and have the holdback
  if (mState != Mounted && mState != Launching) {
    setForce(Vector6::zeros());
    return;
  }

  // Query the catapult, write the result into the attached frame
  // ... yes this function works through sideffects ... :-/
  real_type catLen;
  if (!computeCatFrame(taskInfo.getTime(), catLen)) {
    setForce(Vector6::zeros());
    return;
  }

  // The catapult direction vector
  Vector3 catPos0 = mCatFrame->posToParent(Vector3::zeros());
  Vector3 catDir = mCatFrame->rotToParent(Vector3::unit(0));

  // The launchbar's tip position
  Vector3 lbTip(cos(mAngle)*mLength, 0, -sin(mAngle)*mLength);
  
  // The tension applied over the launchbar
  // allways pulls the aircraft back to the cat ...
  // project the launchbar tip to the catpult line ...
  Vector3 lbTipOnCat = catPos0 + dot(lbTip - catPos0, catDir)*catDir;
  Vector6 force = Vector6::zeros();
  if (mState == Mounted) {
    force.setLinear(mLaunchForce/5*normalize(lbTipOnCat));
  } else {
    force.setLinear(mLaunchForce*normalize(lbTipOnCat));
  }
  
  if (mState == Mounted) {
    // the position of the holdback's deck mount
    Vector3 hbDeckMount = mCatFrame->posToParent(mPosOnCat*Vector3::unit(0));

    // ok, for now the holback is a stiff spring, will model that different
    // when loop closure contranits are availble ...
    Vector3 hbDir = mHoldbackMount - hbDeckMount;
    real_type hbLen = norm(hbDir);
    if (mHoldbackLength < hbLen) {
      Vector3 hbForce = (2*mLaunchForce*(mHoldbackLength - hbLen)/hbLen)*hbDir;
      force += forceFrom(mHoldbackMount, hbForce);
    }

    // Some damping force, just at the position the launchbar applies its force
    force += mLaunchForce/2*mCatFrame->motionToParent(mCatFrame->getRelVel());
  }
  
  setForce(force);
}

void
Launchbar::update(const TaskInfo& taskInfo)
{
  real_type unlagedAngleCommand = mUpAngle;

  // Query the catapult, write the result into the attached frame
  // ... yes this function works through sideffects ... :-/
  real_type catLen;
  if (!computeCatFrame(taskInfo.getTime(), catLen)) {
    mState = Unmounted;
    unlagedAngleCommand = mUpAngle;
  } else {
    // Ok, here we know that the catapult frame contains some sensible values

    // The catapult direction vector
    Vector3 catPos0 = mCatFrame->posToParent(Vector3::zeros());
    Vector3 catDir = mCatFrame->rotToParent(Vector3::unit(0));
    
    real_type angle = computeCurrentLaunchbarAngle();
    // The launchbar's tip position
    Vector3 lbTip(cos(angle)*mLength, 0, -sin(angle)*mLength);
    
    // Compute the distance from the launchbar tip to the catapult
    real_type dist = norm(lbTip-catPos0-dot(catDir, lbTip - catPos0)*catDir);
    
    if (mState == Unmounted) {
      bool tryMount = mTryMountPort.getRealValue();
      if (tryMount) {
        unlagedAngleCommand = mDownAngle;
        
        // can only mount if we are near enough
        if (dist < 0.1) {
          // Now compute a reference position on the catapult line which
          // will be used as the reference for the holdback
          
          // compute the nearest point on the catapult line to the holdback
          // mount
          Vector3 hbNearest = catPos0
            + dot(mHoldbackMount - catPos0, catDir)*catDir;
          
          // Find the distance backwards from that point matching
          // the holdback length
          real_type sqrCatx = mHoldbackLength*mHoldbackLength
            - dot(hbNearest, hbNearest);
          /// There is something wrong if the holdback mount is too far
          if (0 <= sqrCatx) {
            Vector3 hbDeckMount = hbNearest - sqrt(sqrCatx)*catDir;
            
            // The reference position
            mPosOnCat = dot(catDir, hbDeckMount - catPos0);
            //           if (mPosOnCat < 30)
            
            // Ok, survived, mounted now ...
            mState = Mounted;
          }
        }
      } else
        unlagedAngleCommand = mUpAngle;
      
    } else if (mState == Mounted) {
      if (mLaunchCommandPort.getRealValue())
        mState = Launching;
      
      if (dist > 1)
        mState = Unmounted;
      
      unlagedAngleCommand = mDownAngle;
    } else {
      // Launching
      if (dist > 1)
        mState = Unmounted;
      
      Vector3 catPos1 = mCatFrame->posToParent(catLen*Vector3::unit(0));
      if (dot(catPos1, catDir) < 0)
        mState = Unmounted;
      
      unlagedAngleCommand = mDownAngle;
    }
  }

  real_type angleError = unlagedAngleCommand - mAngleCommand;
  angleError = sign(angleError)*min(mAngularVelocity, 40*fabs(angleError));
  /// FIXME: isPerTimestep sample times do not contain the step size ...
  /// hardwire that ATM
  mAngleCommand += 1/120.0*angleError;
}

void
Launchbar::setDiscreteState(const StateStream& state)
{
  state.readSubState(mAngleCommand);
}

void
Launchbar::getDiscreteState(StateStream& state) const
{
  state.writeSubState(mAngleCommand);
}

void
Launchbar::setEnvironment(Environment* environment)
{
  mEnvironment = environment;
}

void
Launchbar::getGround(real_type t)
{
  // Get the position of the contact in the reference system.
  Vector3 pos = mMountFrame->getRefPosition();
  // Query for the ground parameters at this point.
  mGroundVal = mEnvironment->getGround()->getGroundPlane(t, pos);
}

bool
Launchbar::computeCatFrame(real_type t, real_type& catLen)
{
  Vector3 refPos = mMountFrame->getRefPosition();
  CatapultValues catVals;
  const Ground* ground = mEnvironment->getGround();
  // Early return if no cat in range
  if (!ground->getCatapultValues(t, refPos, catVals))
    return false;

  mCatFrame->setRefPosition(catVals.position);
  mCatFrame->setRefOrientation(catVals.orientation);
  Vector6 locVel(mCatFrame->rotFromRef(catVals.velocity.getAngular()),
                 mCatFrame->rotFromRef(catVals.velocity.getLinear()));
  mCatFrame->setRefVel(locVel);
  catLen = catVals.length;

  return true;
}

real_type
Launchbar::computeCurrentLaunchbarAngle(void)
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
  if (fabs(n(2)) <= Limits<real_type>::min())
    return mAngleCommand;

  // that leads to a quadratic equation where we pick the solution pointing
  // backwards:
  real_type nx2nz2 = n(0)*n(0) + n(2)*n(2);

  // we need to didive through that later, with exact operations it should
  // be safe to not check that because of n(2) being bounded away from zero,
  // but due to the square the value can underflow
  if (fabs(nx2nz2) <= Limits<real_type>::min())
    return mAngleCommand;

  // the discriminant (rought german translation ...)
  real_type discr = nx2nz2*mLength*mLength - d*d;
  // Unconstraint angle position
  if (discr <= 0)
    return mAngleCommand;

  // the x coorinate of the tip
  real_type x = -(d*n(0) - fabs(n(2))*sqrt(discr))/nx2nz2;

  // get the z coordinate of the tip from the plane equation
  real_type z = -(d + x*n(0))/n(2);

  // ok, now the angle ...
  real_type angle = atan2(-z, x);
  // limit to the range of movement
  if (angle < mAngleCommand)
    angle = mAngleCommand;
  if (mUpAngle < angle)
    angle = mUpAngle;

  return angle;
}

} // namespace OpenFDM
