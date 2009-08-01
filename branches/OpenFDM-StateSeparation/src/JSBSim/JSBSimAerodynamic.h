/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_JSBSimAerodynamic_H
#define OpenFDM_JSBSimAerodynamic_H

#include <OpenFDM/ConstModel.h>
#include <OpenFDM/ExternalInteract.h>
#include <OpenFDM/Group.h>
#include <OpenFDM/GroupInput.h>
#include <OpenFDM/GroupMechanicLink.h>
#include <OpenFDM/GroupOutput.h>
#include <OpenFDM/MachNumber.h>
#include <OpenFDM/Referenced.h>
#include <OpenFDM/SharedPtr.h>
#include <OpenFDM/WindAxis.h>
#include <OpenFDM/WindAxisForce.h>

namespace OpenFDM {

class JSBSimAerodynamic : public Referenced {
public:
  JSBSimAerodynamic(const std::string& name);
  virtual ~JSBSimAerodynamic(void);

  void setPosition(const Vector3& position);

  void setWingArea(const real_type& wingArea);
  const Port* getWingAreaPort(void);

  void setWingSpan(const real_type& wingSpan);
  const Port* getWingSpanPort(void);

  void setChord(const real_type& chord);
  const Port* getChordPort(void);

  void setWingIncidence(const real_type& wingIncidence);
  const Port* getWingIncidencePort(void);

  void setHTailArea(const real_type& hTailArea);
  const Port* getHTailAreaPort(void);

  void setHTailArm(const real_type& hTailArm);
  const Port* getHTailArmPort(void);

  void setVTailArea(const real_type& vTailArea);
  const Port* getVTailAreaPort(void);

  void setVTailArm(const real_type& vTailArm);
  const Port* getVTailArmPort(void);

  const Port* getAlphaPort(void);
  const Port* getAlphaDotPort(void);
  const Port* getBetaPort(void);
  const Port* getBetaDotPort(void);
  const Port* getTrueAirSpeedPort(void);
  const Port* getCalibratedAirSpeedPort(void);
  const Port* getEquivalentAirSpeedPort(void);
  const Port* getMachPort(void);
  const Port* getDynamicPressurePort(void);
  const Port* getStaticPressurePort(void);
  const Port* getTemperaturePort(void);

  const Port* getPPort(void);
  const Port* getQPort(void);
  const Port* getRPort(void);

  const Port* getUPort(void);
  const Port* getVPort(void);
  const Port* getWPort(void);

  const Port* getWingSpanOver2SpeedPort(void);
  const Port* getChordOver2SpeedPort(void);
  const Port* getHOverWingSpanPort(void);

  const Port* getMechanicLink(void);

  const Port* getDragPort(void);
  const Port* getSidePort(void);
  const Port* getLiftPort(void);

  const Port* getRollPort(void);
  const Port* getPitchPort(void);
  const Port* getYawPort(void);

  Group* getGroup()
  { return mGroup; }

private:
  SharedPtr<Group> mGroup;

  SharedPtr<ConstModel> mWingAreaModel;
  SharedPtr<GroupOutput> mWingAreaOutputModel;

  SharedPtr<ConstModel> mWingSpanModel;
  SharedPtr<GroupOutput> mWingSpanOutputModel;

  SharedPtr<ConstModel> mChordModel;
  SharedPtr<GroupOutput> mChordOutputModel;

  SharedPtr<ConstModel> mWingIncidenceModel;
  SharedPtr<GroupOutput> mWingIncidenceOutputModel;

  SharedPtr<ConstModel> mHTailAreaModel;
  SharedPtr<GroupOutput> mHTailAreaOutputModel;

  SharedPtr<ConstModel> mHTailArmModel;
  SharedPtr<GroupOutput> mHTailArmOutputModel;

  SharedPtr<ConstModel> mVTailAreaModel;
  SharedPtr<GroupOutput> mVTailAreaOutputModel;

  SharedPtr<ConstModel> mVTailArmModel;
  SharedPtr<GroupOutput> mVTailArmOutputModel;

  SharedPtr<GroupOutput> mAlphaOutputModel;
  SharedPtr<GroupOutput> mAlphaDotOutputModel;
  SharedPtr<GroupOutput> mBetaOutputModel;
  SharedPtr<GroupOutput> mBetaDotOutputModel;
  SharedPtr<GroupOutput> mTrueAirSpeedOutputModel;
  SharedPtr<GroupOutput> mCalibratedAirSpeedOutputModel;
  SharedPtr<GroupOutput> mEquivalentAirSpeedOutputModel;
  SharedPtr<GroupOutput> mMachOutputModel;
  SharedPtr<GroupOutput> mDynamicPressureOutputModel;
  SharedPtr<GroupOutput> mStaticPressureOutputModel;
  SharedPtr<GroupOutput> mTemperatureOutputModel;
//   SharedPtr<GroupOutput> mAltitudeOutputModel;
//   SharedPtr<GroupOutput> mAGLOutputModel;

  SharedPtr<GroupOutput> mPOutputModel;
  SharedPtr<GroupOutput> mQOutputModel;
  SharedPtr<GroupOutput> mROutputModel;

  SharedPtr<GroupOutput> mUOutputModel;
  SharedPtr<GroupOutput> mVOutputModel;
  SharedPtr<GroupOutput> mWOutputModel;

  SharedPtr<GroupOutput> mWingSpanOver2SpeedOutputModel;
  SharedPtr<GroupOutput> mChordOver2SpeedOutputModel;
  SharedPtr<GroupOutput> mHOverWingSpanOutputModel;

  SharedPtr<GroupMechanicLink> mMechanicLinkModel;
  SharedPtr<ExternalInteract> mExternalInteract;

  SharedPtr<GroupInput> mSideInputModel;
  SharedPtr<GroupInput> mDragInputModel;
  SharedPtr<GroupInput> mLiftInputModel;
  SharedPtr<GroupInput> mRollInputModel;
  SharedPtr<GroupInput> mPitchInputModel;
  SharedPtr<GroupInput> mYawInputModel;
};

} //namespace OpenFDM

#endif
