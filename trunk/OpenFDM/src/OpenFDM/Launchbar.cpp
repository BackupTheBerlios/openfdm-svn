/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"
#include "Environment.h"
#include "Launchbar.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Launchbar, ExternalForce)
  END_OPENFDM_OBJECT_DEF

Launchbar::Launchbar(const std::string& name)
  : ExternalForce(name),
    mLength(0.6),
    mHoldbackLength(1.5),
    mHoldBackMount(0, 0, 0.1),
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
                      << "\" is not connected!" << endl;
    return false;
  }

  mLaunchCommandPort = getInputPort(1)->toRealPortHandle();
  if (!mLaunchCommandPort.isConnected()) {
    Log(Model, Error) << "Initialization of Launchbar model \"" << getName()
                      << "\" failed: Input port \"" << getInputPortName(1)
                      << "\" is not connected!" << endl;
    return false;
  }

  mEnvironment = getEnvironment();
  if (!mEnvironment)
    return false;
  return ExternalForce::init();
}

void
Launchbar::output(const TaskInfo& taskInfo)
{
  if (nonZeroIntersection(taskInfo.getSampleTimeSet(),
                          SampleTime::PerTimestep)) {
    Log(Model, Debug) << "Launchbar::output(): \"" << getName()
                      << "\" computing ground plane below" << endl;
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
  Vector3 catDir = mCatFrame->rotToParent(Vector3::unit(1));

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
    Vector3 hbDeckMount = mCatFrame->posToParent(mPosOnCat*Vector3::unit(1));

    // ok, for now the holback is a stiff spring, will model that different
    // when loop closure contranits are availble ...
    Vector3 hbDir = mHoldBackMount - hbDeckMount;
    real_type hbLen = norm(hbDir);
    if (mHoldbackLength < hbLen) {
      Vector3 hbForce = (2*mLaunchForce*(mHoldbackLength - hbLen)/hbLen)*hbDir;
      force += forceFrom(mHoldBackMount, hbForce);
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
    Vector3 catDir = mCatFrame->rotToParent(Vector3::unit(1));
    
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
            + dot(mHoldBackMount - catPos0, catDir)*catDir;
          
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
      
      Vector3 catPos1 = mCatFrame->posToParent(catLen*Vector3::unit(1));
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
Launchbar::getGround(real_type t)
{
  // FIXME
  if (!mEnvironment) {
    mEnvironment = getEnvironment();
  }

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
  
  // Get the distance to ground
  // negative values are above ground
  real_type distToGround = lp.getDist(Vector3::zeros());
  // The angle between the local x-axis and the launchbar, positive upwards
  real_type aCosAngle = distToGround/mLength;
  if (aCosAngle < -1)
    aCosAngle = -1;
  if (1 < aCosAngle)
    aCosAngle = 1;
  /// FIXME: could be done different ????
  real_type angle = - acos(aCosAngle) + pi05;
  // limit to the range of movement
  if (angle < mAngleCommand)
    angle = mAngleCommand;
  if (mUpAngle < angle)
    angle = mUpAngle;

  return angle;
}

} // namespace OpenFDM
