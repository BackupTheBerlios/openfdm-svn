/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
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
  mGroup->connect(mWingAreaModel->getPort("output"),
                  mWingAreaOutputModel->getPort("input"));

  mWingSpanModel = new ConstModel("Wing Span Constant", 0);
  mGroup->addChild(mWingSpanModel);
  mWingSpanOutputModel = new GroupOutput("Wing Span Output");
  mGroup->addChild(mWingSpanOutputModel);
  mGroup->connect(mWingSpanModel->getPort("output"),
                  mWingSpanOutputModel->getPort("input"));

  mChordModel = new ConstModel("Chord Constant", 0);
  mGroup->addChild(mChordModel);
  mChordOutputModel = new GroupOutput("Chord Output");
  mGroup->addChild(mChordOutputModel);
  mGroup->connect(mChordModel->getPort("output"),
                  mChordOutputModel->getPort("input"));

  mWingIncidenceModel = new ConstModel("Wing Incidence Constant", 0);
  mGroup->addChild(mWingIncidenceModel);
  mWingIncidenceOutputModel = new GroupOutput("Wing Incidence Output");
  mGroup->addChild(mWingIncidenceOutputModel);
  mGroup->connect(mWingIncidenceModel->getPort("output"),
                  mWingIncidenceOutputModel->getPort("input"));

  mHTailAreaModel = new ConstModel("HTailArea Constant", 0);
  mGroup->addChild(mHTailAreaModel);
  mHTailAreaOutputModel = new GroupOutput("HTailArea Output");
  mGroup->addChild(mHTailAreaOutputModel);
  mGroup->connect(mHTailAreaModel->getPort("output"),
                  mHTailAreaOutputModel->getPort("input"));

  mHTailArmModel = new ConstModel("HTailArm Constant", 0);
  mGroup->addChild(mHTailArmModel);
  mHTailArmOutputModel = new GroupOutput("HTailArm Output");
  mGroup->addChild(mHTailArmOutputModel);
  mGroup->connect(mHTailArmModel->getPort("output"),
                  mHTailArmOutputModel->getPort("input"));


  mVTailAreaModel = new ConstModel("VTailArea Constant", 0);
  mGroup->addChild(mVTailAreaModel);
  mVTailAreaOutputModel = new GroupOutput("VTailArea Output");
  mGroup->addChild(mVTailAreaOutputModel);
  mGroup->connect(mVTailAreaModel->getPort("output"),
                  mVTailAreaOutputModel->getPort("input"));

  mVTailArmModel = new ConstModel("VTailArm Constant", 0);
  mGroup->addChild(mVTailArmModel);
  mVTailArmOutputModel = new GroupOutput("VTailArm Output");
  mGroup->addChild(mVTailArmOutputModel);
  mGroup->connect(mVTailArmModel->getPort("output"),
                  mVTailArmOutputModel->getPort("input"));


  mMechanicLinkModel = new GroupMechanicLink("Mechanic Link Model");

  mExternalInteract = new ExternalInteract("ExternalInteract");
//   mExternalInteract->setPosition(mass->getPosition());
  mGroup->addChild(mExternalInteract);

  mGroup->connect(mMechanicLinkModel->getPort("link"),
                  mExternalInteract->getPort("link"));

  mExternalInteract->setEnableBodyWindVelocity(true);
  mExternalInteract->setEnableDensity(true);
  mExternalInteract->setEnableStaticPressure(true);
  mExternalInteract->setEnableTemperature(true);
  mExternalInteract->setEnableSoundSpeed(true);
  mExternalInteract->setEnableAltitude(true);

  DynamicPressure* dynamicPressure = new DynamicPressure("DynamicPressure");
  mGroup->addChild(dynamicPressure);

  mGroup->connect(mExternalInteract->getPort("bodyWindVelocity"),
                  dynamicPressure->getPort("velocity"));
  mGroup->connect(mExternalInteract->getPort("density"),
                  dynamicPressure->getPort("density"));


  MachNumber* machNumber = new MachNumber("MachNumber");
  mGroup->addChild(machNumber);

  mGroup->connect(mExternalInteract->getPort("bodyWindVelocity"),
                  machNumber->getPort("velocity"));
  mGroup->connect(mExternalInteract->getPort("soundSpeed"),
                  machNumber->getPort("soundSpeed"));


  WindAxis* windAxis = new WindAxis("WindAxis");
  mGroup->addChild(windAxis);
  mGroup->connect(mExternalInteract->getPort("bodyWindVelocity"),
                  windAxis->getPort("bodyVelocity"));


  WindAxisForce* windAxisForce = new WindAxisForce("WindAxisForce");
  mGroup->addChild(windAxisForce);

  mGroup->connect(windAxis->getPort("alpha"),
                  windAxisForce->getPort("alpha"));
  mGroup->connect(windAxis->getPort("beta"),
                  windAxisForce->getPort("beta"));

  mGroup->connect(windAxis->getPort("bodyForce"),
                  mExternalInteract->getPort("bodyForce"));


  mAlphaOutputModel = new GroupOutput("Alpha Output");
  mGroup->addChild(mAlphaOutputModel);
  mGroup->connect(windAxis->getPort("alpha"),
                  mAlphaOutputModel->getPort("input"));

  mAlphaDotOutputModel = new GroupOutput("AlphaDot Output");
  mGroup->addChild(mAlphaOutputModel);
  mGroup->connect(windAxis->getPort("alphaDot"),
                  mAlphaDotOutputModel->getPort("input"));

  mBetaOutputModel = new GroupOutput("Beta Output");
  mGroup->addChild(mBetaOutputModel);
  mGroup->connect(windAxis->getPort("beta"),
                  mBetaOutputModel->getPort("input"));

  mBetaDotOutputModel = new GroupOutput("BetaDot Output");
  mGroup->addChild(mBetaOutputModel);
  mGroup->connect(windAxis->getPort("betaDot"),
                  mBetaDotOutputModel->getPort("input"));

  mTrueAirSpeedOutputModel = new GroupOutput("True Air Speed Output");
  mGroup->addChild(mTrueAirSpeedOutputModel);
  mGroup->connect(windAxis->getPort("airSpeed"),
                  mTrueAirSpeedOutputModel->getPort("input"));

  mCalibratedAirSpeedOutputModel = new GroupOutput("Calibrated Air Speed Output");
  mGroup->addChild(mCalibratedAirSpeedOutputModel);
  /// FIXME wrong
  mGroup->connect(windAxis->getPort("airSpeed"),
                  mCalibratedAirSpeedOutputModel->getPort("input"));

  mEquivalentAirSpeedOutputModel = new GroupOutput("Equivalent Air Speed Output");
  mGroup->addChild(mEquivalentAirSpeedOutputModel);
  /// FIXME wrong
  mGroup->connect(windAxis->getPort("airSpeed"),
                  mEquivalentAirSpeedOutputModel->getPort("input"));


  mMachOutputModel = new GroupOutput("Mach Output");
  mGroup->addChild(mMachOutputModel);
  mGroup->connect(machNumber->getPort("mach"),
                  mMachOutputModel->getPort("input"));

  mDynamicPressureOutputModel = new GroupOutput("Dynamic Pressure Output");
  mGroup->addChild(mDynamicPressureOutputModel);
  mGroup->connect(dynamicPressure->getPort("dynamicPressure"),
                  mDynamicPressureOutputModel->getPort("input"));

  mStaticPressureOutputModel = new GroupOutput("Static Pressure Output");
  mGroup->addChild(mStaticPressureOutputModel);
  mGroup->connect(mExternalInteract->getPort("staticPressure"),
                  mStaticPressureOutputModel->getPort("input"));

  mTemperatureOutputModel = new GroupOutput("Temperature Output");
  mGroup->addChild(mTemperatureOutputModel);
  mGroup->connect(mExternalInteract->getPort("temperature"),
                  mTemperatureOutputModel->getPort("input"));

  
  MatrixSplit* linearVel = new MatrixSplit("Linear Velocity Split");
  mGroup->connect(mExternalInteract->getPort("bodyWindVelocity"),
                  linearVel->getPort("input"));
  linearVel->addOutputPort("u");
  linearVel->addOutputPort("v");
  linearVel->addOutputPort("w");

  mUOutputModel = new GroupOutput("U Output");
  mGroup->addChild(mUOutputModel);
  mGroup->connect(linearVel->getPort("u"),
                  mUOutputModel->getPort("input"));
  mVOutputModel = new GroupOutput("V Output");
  mGroup->addChild(mVOutputModel);
  mGroup->connect(linearVel->getPort("v"),
                  mVOutputModel->getPort("input"));
  mWOutputModel = new GroupOutput("W Output");
  mGroup->addChild(mWOutputModel);
  mGroup->connect(linearVel->getPort("w"),
                  mWOutputModel->getPort("input"));

  MatrixSplit* angularVel = new MatrixSplit("Angular Velocity Split");
  /// FIXME make wind angular velocity sensable
  mGroup->connect(mExternalInteract->getPort("bodyAngularVelocity"),
                  angularVel->getPort("input"));
  angularVel->addOutputPort("p");
  angularVel->addOutputPort("q");
  angularVel->addOutputPort("r");

  mPOutputModel = new GroupOutput("P Output");
  mGroup->addChild(mPOutputModel);
  mGroup->connect(angularVel->getPort("p"),
                  mPOutputModel->getPort("input"));
  mQOutputModel = new GroupOutput("Q Output");
  mGroup->addChild(mQOutputModel);
  mGroup->connect(angularVel->getPort("q"),
                  mQOutputModel->getPort("input"));
  mROutputModel = new GroupOutput("R Output");
  mGroup->addChild(mQOutputModel);
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
  // FIXME??
//   mGroup->connect(mExternalInteract->getPort("altitude"),
//                   hOverWingSpan->getPort("input1"));
  mGroup->connect(mExternalInteract->getPort("aboveGroundLevel"),
                  hOverWingSpan->getPort("input1"));

  mHOverWingSpanOutputModel = new GroupOutput("HOverWingSpan Output");
  mGroup->addChild(mHOverWingSpanOutputModel);
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

  mGroup->connect(torque->getPort("output"),
                  mExternalInteract->getPort("bodyTorque"));
}

JSBSimAerodynamic::~JSBSimAerodynamic(void)
{
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
JSBSimAerodynamic::setWingIncidence(const real_type& wingIncidence)
{
  mWingIncidenceModel->setScalarValue(wingIncidence);
}

const Port*
JSBSimAerodynamic::getWingIncidencePort(void)
{
  return mGroup->getPort(mWingIncidenceOutputModel->getExternalPortIndex());
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
