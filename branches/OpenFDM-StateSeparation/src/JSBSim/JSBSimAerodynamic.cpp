/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "JSBSimAerodynamic.h"

#include <OpenFDM/DynamicPressure.h>
#include <OpenFDM/Gain.h>
#include <OpenFDM/Product.h>
#include <OpenFDM/SafeReciprocal.h>
#include <OpenFDM/MatrixConcat.h>
#include <OpenFDM/MatrixSplit.h>

namespace OpenFDM {

JSBSimAerodynamic::JSBSimAerodynamic(const std::string& name) :
  mGroup(new Group(name))
{
  mWingAreaModel = new ConstModel("Wing Area Constant", 0);
  mGroup->addChild(mWingAreaModel);
  mWingAreaOutputModel = new GroupOutput("Wing Area Output");
  mGroup->addChild(mWingAreaOutputModel);
  mWingAreaOutputModel->setExternalPortName("wingArea");
  mGroup->connect(mWingAreaModel->getPort("output"),
                  mWingAreaOutputModel->getPort("input"));

  mWingSpanModel = new ConstModel("Wing Span Constant", 0);
  mGroup->addChild(mWingSpanModel);
  mWingSpanOutputModel = new GroupOutput("Wing Span Output");
  mGroup->addChild(mWingSpanOutputModel);
  mWingSpanOutputModel->setExternalPortName("wingSpan");
  mGroup->connect(mWingSpanModel->getPort("output"),
                  mWingSpanOutputModel->getPort("input"));

  mChordModel = new ConstModel("Chord Constant", 0);
  mGroup->addChild(mChordModel);
  mChordOutputModel = new GroupOutput("Chord Output");
  mGroup->addChild(mChordOutputModel);
  mChordOutputModel->setExternalPortName("chord");
  mGroup->connect(mChordModel->getPort("output"),
                  mChordOutputModel->getPort("input"));

  mHTailAreaModel = new ConstModel("HTailArea Constant", 0);
  mGroup->addChild(mHTailAreaModel);
  mHTailAreaOutputModel = new GroupOutput("HTailArea Output");
  mGroup->addChild(mHTailAreaOutputModel);
  mHTailAreaOutputModel->setExternalPortName("hTailArea");
  mGroup->connect(mHTailAreaModel->getPort("output"),
                  mHTailAreaOutputModel->getPort("input"));

  mHTailArmModel = new ConstModel("HTailArm Constant", 0);
  mGroup->addChild(mHTailArmModel);
  mHTailArmOutputModel = new GroupOutput("HTailArm Output");
  mGroup->addChild(mHTailArmOutputModel);
  mHTailArmOutputModel->setExternalPortName("hTailArm");
  mGroup->connect(mHTailArmModel->getPort("output"),
                  mHTailArmOutputModel->getPort("input"));


  mVTailAreaModel = new ConstModel("VTailArea Constant", 0);
  mGroup->addChild(mVTailAreaModel);
  mVTailAreaOutputModel = new GroupOutput("VTailArea Output");
  mGroup->addChild(mVTailAreaOutputModel);
  mVTailAreaOutputModel->setExternalPortName("vTailArea");
  mGroup->connect(mVTailAreaModel->getPort("output"),
                  mVTailAreaOutputModel->getPort("input"));

  mVTailArmModel = new ConstModel("VTailArm Constant", 0);
  mGroup->addChild(mVTailArmModel);
  mVTailArmOutputModel = new GroupOutput("VTailArm Output");
  mGroup->addChild(mVTailArmOutputModel);
  mVTailArmOutputModel->setExternalPortName("vTailArm");
  mGroup->connect(mVTailArmModel->getPort("output"),
                  mVTailArmOutputModel->getPort("input"));

  mMechanicLinkModel = new GroupMechanicLink("Mechanic Link Model");
  mGroup->addChild(mMechanicLinkModel);

  mExternalInteract = new ExternalInteract("ExternalInteract");
  mGroup->addChild(mExternalInteract);

  mGroup->connect(mMechanicLinkModel->getPort("link"),
                  mExternalInteract->getPort("link"));

  mExternalInteract->setEnableLinearWindVelocity(true);
  mExternalInteract->setEnableDensity(true);
  mExternalInteract->setEnableStaticPressure(true);
  mExternalInteract->setEnableTemperature(true);
  mExternalInteract->setEnableSoundSpeed(true);
  mExternalInteract->setEnableForce(true);

  DynamicPressure* dynamicPressure = new DynamicPressure("DynamicPressure");
  mGroup->addChild(dynamicPressure);

  mGroup->connect(mExternalInteract->getPort("linearWindVelocity"),
                  dynamicPressure->getPort("velocity"));
  mGroup->connect(mExternalInteract->getPort("density"),
                  dynamicPressure->getPort("density"));


  MachNumber* machNumber = new MachNumber("MachNumber");
  mGroup->addChild(machNumber);

  mGroup->connect(mExternalInteract->getPort("linearWindVelocity"),
                  machNumber->getPort("velocity"));
  mGroup->connect(mExternalInteract->getPort("soundSpeed"),
                  machNumber->getPort("soundSpeed"));


  WindAxis* windAxis = new WindAxis("WindAxis");
  mGroup->addChild(windAxis);
  mGroup->connect(mExternalInteract->getPort("linearWindVelocity"),
                  windAxis->getPort("bodyVelocity"));


  WindAxisForce* windAxisForce = new WindAxisForce("WindAxisForce");
  mGroup->addChild(windAxisForce);

  mGroup->connect(windAxis->getPort("alpha"),
                  windAxisForce->getPort("alpha"));
  mGroup->connect(windAxis->getPort("beta"),
                  windAxisForce->getPort("beta"));

  mGroup->connect(windAxisForce->getPort("bodyForce"),
                  mExternalInteract->getPort("force"));


  mAlphaOutputModel = new GroupOutput("Alpha Output");
  mGroup->addChild(mAlphaOutputModel);
  mAlphaOutputModel->setExternalPortName("alpha");
  mGroup->connect(windAxis->getPort("alpha"),
                  mAlphaOutputModel->getPort("input"));

  mAlphaDotOutputModel = new GroupOutput("AlphaDot Output");
  mGroup->addChild(mAlphaDotOutputModel);
  mAlphaDotOutputModel->setExternalPortName("alphaDot");
  mGroup->connect(windAxis->getPort("alphaDot"),
                  mAlphaDotOutputModel->getPort("input"));

  mBetaOutputModel = new GroupOutput("Beta Output");
  mGroup->addChild(mBetaOutputModel);
  mBetaOutputModel->setExternalPortName("beta");
  mGroup->connect(windAxis->getPort("beta"),
                  mBetaOutputModel->getPort("input"));

  mBetaDotOutputModel = new GroupOutput("BetaDot Output");
  mGroup->addChild(mBetaDotOutputModel);
  mBetaDotOutputModel->setExternalPortName("betaDot");
  mGroup->connect(windAxis->getPort("betaDot"),
                  mBetaDotOutputModel->getPort("input"));

  mTrueAirSpeedOutputModel = new GroupOutput("True Air Speed Output");
  mGroup->addChild(mTrueAirSpeedOutputModel);
  mTrueAirSpeedOutputModel->setExternalPortName("airSpeed");
  mGroup->connect(windAxis->getPort("airSpeed"),
                  mTrueAirSpeedOutputModel->getPort("input"));

  mGroundSpeedOutputModel = new GroupOutput("Ground Speed Output");
  mGroup->addChild(mGroundSpeedOutputModel);
  mGroundSpeedOutputModel->setExternalPortName("groundSpeed");
  mExternalInteract->setEnableGroundSpeed(true);
  mGroup->connect(mExternalInteract->getPort("groundSpeed"),
                  mGroundSpeedOutputModel->getPort("input"));

  mClimbSpeedOutputModel = new GroupOutput("Climb Speed Output");
  mGroup->addChild(mClimbSpeedOutputModel);
  mClimbSpeedOutputModel->setExternalPortName("climbSpeed");
  mExternalInteract->setEnableClimbSpeed(true);
  mGroup->connect(mExternalInteract->getPort("climbSpeed"),
                  mClimbSpeedOutputModel->getPort("input"));

  mCalibratedAirSpeedOutputModel = new GroupOutput("Calibrated Air Speed Output");
  mGroup->addChild(mCalibratedAirSpeedOutputModel);
  mCalibratedAirSpeedOutputModel->setExternalPortName("calibratedAirSpeed");
  /// FIXME wrong
  mGroup->connect(windAxis->getPort("airSpeed"),
                  mCalibratedAirSpeedOutputModel->getPort("input"));

  mEquivalentAirSpeedOutputModel = new GroupOutput("Equivalent Air Speed Output");
  mGroup->addChild(mEquivalentAirSpeedOutputModel);
  mEquivalentAirSpeedOutputModel->setExternalPortName("equivalentAirSpeed");
  /// FIXME wrong
  mGroup->connect(windAxis->getPort("airSpeed"),
                  mEquivalentAirSpeedOutputModel->getPort("input"));


  mMachOutputModel = new GroupOutput("Mach Output");
  mGroup->addChild(mMachOutputModel);
  mMachOutputModel->setExternalPortName("machNumber");
  mGroup->connect(machNumber->getPort("machNumber"),
                  mMachOutputModel->getPort("input"));

  mDynamicPressureOutputModel = new GroupOutput("Dynamic Pressure Output");
  mGroup->addChild(mDynamicPressureOutputModel);
  mDynamicPressureOutputModel->setExternalPortName("dynamicPressure");
  mGroup->connect(dynamicPressure->getPort("dynamicPressure"),
                  mDynamicPressureOutputModel->getPort("input"));

  mStaticPressureOutputModel = new GroupOutput("Static Pressure Output");
  mGroup->addChild(mStaticPressureOutputModel);
  mStaticPressureOutputModel->setExternalPortName("staticPressure");
  mGroup->connect(mExternalInteract->getPort("staticPressure"),
                  mStaticPressureOutputModel->getPort("input"));

  mTemperatureOutputModel = new GroupOutput("Temperature Output");
  mGroup->addChild(mTemperatureOutputModel);
  mTemperatureOutputModel->setExternalPortName("temperature");
  mGroup->connect(mExternalInteract->getPort("temperature"),
                  mTemperatureOutputModel->getPort("input"));

  
  MatrixSplit* linearVel = new MatrixSplit("Linear Velocity Split");
  mGroup->addChild(linearVel);
  mExternalInteract->setEnableLinearWindVelocity(true);
  mGroup->connect(mExternalInteract->getPort("linearWindVelocity"),
                  linearVel->getPort("input"));
  linearVel->addOutputPort("u");
  linearVel->addOutputPort("v");
  linearVel->addOutputPort("w");

  mUOutputModel = new GroupOutput("U Output");
  mGroup->addChild(mUOutputModel);
  mUOutputModel->setExternalPortName("u");
  mGroup->connect(linearVel->getPort("u"),
                  mUOutputModel->getPort("input"));
  mVOutputModel = new GroupOutput("V Output");
  mGroup->addChild(mVOutputModel);
  mVOutputModel->setExternalPortName("v");
  mGroup->connect(linearVel->getPort("v"),
                  mVOutputModel->getPort("input"));
  mWOutputModel = new GroupOutput("W Output");
  mGroup->addChild(mWOutputModel);
  mWOutputModel->setExternalPortName("w");
  mGroup->connect(linearVel->getPort("w"),
                  mWOutputModel->getPort("input"));

  MatrixSplit* angularVel = new MatrixSplit("Angular Velocity Split");
  mGroup->addChild(angularVel);
  mExternalInteract->setEnableAngularWindVelocity(true);
  mGroup->connect(mExternalInteract->getPort("angularWindVelocity"),
                  angularVel->getPort("input"));
  angularVel->addOutputPort("p");
  angularVel->addOutputPort("q");
  angularVel->addOutputPort("r");

  mPOutputModel = new GroupOutput("P Output");
  mGroup->addChild(mPOutputModel);
  mPOutputModel->setExternalPortName("p");
  mGroup->connect(angularVel->getPort("p"),
                  mPOutputModel->getPort("input"));
  mQOutputModel = new GroupOutput("Q Output");
  mGroup->addChild(mQOutputModel);
  mQOutputModel->setExternalPortName("q");
  mGroup->connect(angularVel->getPort("q"),
                  mQOutputModel->getPort("input"));
  mROutputModel = new GroupOutput("R Output");
  mGroup->addChild(mROutputModel);
  mROutputModel->setExternalPortName("r");
  mGroup->connect(angularVel->getPort("r"),
                  mROutputModel->getPort("input"));

  Gain* twoSpeed = new Gain("Two Speed", 2);
  mGroup->addChild(twoSpeed);
  mGroup->connect(windAxis->getPort("airSpeed"),
                  twoSpeed->getPort("input"));

  SafeReciprocal* rec2Speed = new SafeReciprocal("one over 2 speed");
  mGroup->addChild(rec2Speed);
  mGroup->connect(twoSpeed->getPort("output"),
                  rec2Speed->getPort("input"));

  Product* wingSpanOver2Speed = new Product("WingSpanOver2Speed");
  mGroup->addChild(wingSpanOver2Speed);
  wingSpanOver2Speed->setNumFactors(2);
  mGroup->connect(rec2Speed->getPort("output"),
                  wingSpanOver2Speed->getPort("input0"));
  mGroup->connect(mWingSpanModel->getPort("output"),
                  wingSpanOver2Speed->getPort("input1"));

  mWingSpanOver2SpeedOutputModel = new GroupOutput("WingSpanOver2Speed Output");
  mGroup->addChild(mWingSpanOver2SpeedOutputModel);
  mWingSpanOver2SpeedOutputModel->setExternalPortName("wingSpanOver2Speed");
  mGroup->connect(wingSpanOver2Speed->getPort("output"),
                  mWingSpanOver2SpeedOutputModel->getPort("input"));

  Product* chordOver2Speed = new Product("ChordOver2Speed");
  mGroup->addChild(chordOver2Speed);
  chordOver2Speed->setNumFactors(2);
  mGroup->connect(rec2Speed->getPort("output"),
                  chordOver2Speed->getPort("input0"));
  mGroup->connect(mChordModel->getPort("output"),
                  chordOver2Speed->getPort("input1"));

  mChordOver2SpeedOutputModel = new GroupOutput("ChordOver2Speed Output");
  mGroup->addChild(mChordOver2SpeedOutputModel);
  mChordOver2SpeedOutputModel->setExternalPortName("chordOver2Speed");
  mGroup->connect(chordOver2Speed->getPort("output"),
                  mChordOver2SpeedOutputModel->getPort("input"));

  SafeReciprocal* recWingSpan = new SafeReciprocal("one over wing span");
  mGroup->addChild(recWingSpan);
  mGroup->connect(mWingSpanModel->getPort("output"),
                  recWingSpan->getPort("input"));

  Product* hOverWingSpan = new Product("HOverWingSpan");
  mGroup->addChild(hOverWingSpan);
  hOverWingSpan->setNumFactors(2);
  mGroup->connect(recWingSpan->getPort("output"),
                  hOverWingSpan->getPort("input0"));

  mExternalInteract->setEnableAboveGroundLevel(true);
  mGroup->connect(mExternalInteract->getPort("aboveGroundLevel"),
                  hOverWingSpan->getPort("input1"));

  mHOverWingSpanOutputModel = new GroupOutput("HOverWingSpan Output");
  mGroup->addChild(mHOverWingSpanOutputModel);
  mHOverWingSpanOutputModel->setExternalPortName("hOverWingSpan");
  mGroup->connect(hOverWingSpan->getPort("output"),
                  mHOverWingSpanOutputModel->getPort("input"));

  // Inputs
  mDragInputModel = new GroupInput("Drag Input");
  mGroup->addChild(mDragInputModel);

  mSideInputModel = new GroupInput("Side Input");
  mGroup->addChild(mSideInputModel);

  mLiftInputModel = new GroupInput("Lift Input");
  mGroup->addChild(mLiftInputModel);

  mGroup->connect(mDragInputModel->getPort("output"),
                  windAxisForce->getPort("drag"));
  mGroup->connect(mSideInputModel->getPort("output"),
                  windAxisForce->getPort("side"));
  mGroup->connect(mLiftInputModel->getPort("output"),
                  windAxisForce->getPort("lift"));


  mRollInputModel = new GroupInput("Roll Input");
  mGroup->addChild(mRollInputModel);

  mPitchInputModel = new GroupInput("Pitch Input");
  mGroup->addChild(mPitchInputModel);

  mYawInputModel = new GroupInput("Yaw Input");
  mGroup->addChild(mYawInputModel);

  MatrixConcat* torque = new MatrixConcat("Torque");
  mGroup->addChild(torque);

  mGroup->connect(mRollInputModel->getPort("output"),
                  torque->addInputPort("x"));
  mGroup->connect(mPitchInputModel->getPort("output"),
                  torque->addInputPort("y"));
  mGroup->connect(mYawInputModel->getPort("output"),
                  torque->addInputPort("z"));

  mExternalInteract->setEnableTorque(true);
  mGroup->connect(torque->getPort("output"),
                  mExternalInteract->getPort("torque"));
}

JSBSimAerodynamic::~JSBSimAerodynamic(void)
{
}

void
JSBSimAerodynamic::setPosition(const Vector3& position)
{
  mExternalInteract->setPosition(position);
}

void
JSBSimAerodynamic::setWingArea(const real_type& wingArea)
{
  mWingAreaModel->setScalarValue(wingArea);
}

const Port*
JSBSimAerodynamic::getWingAreaPort(void)
{
  return mGroup->getPort(mWingAreaOutputModel->getExternalPortIndex());
}

void
JSBSimAerodynamic::setWingSpan(const real_type& wingSpan)
{
  mWingSpanModel->setScalarValue(wingSpan);
}

const Port*
JSBSimAerodynamic::getWingSpanPort(void)
{
  return mGroup->getPort(mWingSpanOutputModel->getExternalPortIndex());
}

void
JSBSimAerodynamic::setChord(const real_type& chord)
{
  mChordModel->setScalarValue(chord);
}

const Port*
JSBSimAerodynamic::getChordPort(void)
{
  return mGroup->getPort(mChordOutputModel->getExternalPortIndex());
}

void
JSBSimAerodynamic::setHTailArea(const real_type& hTailArea)
{
  mHTailAreaModel->setScalarValue(hTailArea);
}

const Port*
JSBSimAerodynamic::getHTailAreaPort(void)
{
  return mGroup->getPort(mHTailAreaOutputModel->getExternalPortIndex());
}

void
JSBSimAerodynamic::setHTailArm(const real_type& hTailArm)
{
  mHTailArmModel->setScalarValue(hTailArm);
}

const Port*
JSBSimAerodynamic::getHTailArmPort(void)
{
  return mGroup->getPort(mHTailArmOutputModel->getExternalPortIndex());
}

void
JSBSimAerodynamic::setVTailArea(const real_type& vTailArea)
{
  mVTailAreaModel->setScalarValue(vTailArea);
}

const Port*
JSBSimAerodynamic::getVTailAreaPort(void)
{
  return mGroup->getPort(mVTailAreaOutputModel->getExternalPortIndex());
}

void
JSBSimAerodynamic::setVTailArm(const real_type& vTailArm)
{
  mVTailArmModel->setScalarValue(vTailArm);
}

const Port*
JSBSimAerodynamic::getVTailArmPort(void)
{
  return mGroup->getPort(mVTailArmOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getAlphaPort(void)
{
  return mGroup->getPort(mAlphaOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getAlphaDotPort(void)
{
  return mGroup->getPort(mAlphaDotOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getBetaPort(void)
{
  return mGroup->getPort(mBetaOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getBetaDotPort(void)
{
  return mGroup->getPort(mBetaDotOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getTrueAirSpeedPort(void)
{
  return mGroup->getPort(mTrueAirSpeedOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getGroundSpeedPort(void)
{
  return mGroup->getPort(mGroundSpeedOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getClimbSpeedPort(void)
{
  return mGroup->getPort(mClimbSpeedOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getCalibratedAirSpeedPort(void)
{
  return mGroup->getPort(mCalibratedAirSpeedOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getEquivalentAirSpeedPort(void)
{
  return mGroup->getPort(mEquivalentAirSpeedOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getMachPort(void)
{
  return mGroup->getPort(mMachOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getDynamicPressurePort(void)
{
  return mGroup->getPort(mDynamicPressureOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getStaticPressurePort(void)
{
  return mGroup->getPort(mStaticPressureOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getTemperaturePort(void)
{
  return mGroup->getPort(mTemperatureOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getPPort(void)
{
  return mGroup->getPort(mPOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getQPort(void)
{
  return mGroup->getPort(mQOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getRPort(void)
{
  return mGroup->getPort(mROutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getUPort(void)
{
  return mGroup->getPort(mUOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getVPort(void)
{
  return mGroup->getPort(mVOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getWPort(void)
{
  return mGroup->getPort(mWOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getWingSpanOver2SpeedPort(void)
{
  return mGroup->getPort(mWingSpanOver2SpeedOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getChordOver2SpeedPort(void)
{
  return mGroup->getPort(mChordOver2SpeedOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getHOverWingSpanPort(void)
{
  return mGroup->getPort(mHOverWingSpanOutputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getMechanicLink(void)
{
  return mGroup->getPort(mMechanicLinkModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getDragPort(void)
{
  return mGroup->getPort(mDragInputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getSidePort(void)
{
  return mGroup->getPort(mSideInputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getLiftPort(void)
{
  return mGroup->getPort(mLiftInputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getRollPort(void)
{
  return mGroup->getPort(mRollInputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getPitchPort(void)
{
  return mGroup->getPort(mPitchInputModel->getExternalPortIndex());
}

const Port*
JSBSimAerodynamic::getYawPort(void)
{
  return mGroup->getPort(mYawInputModel->getExternalPortIndex());
}

} //namespace OpenFDM
