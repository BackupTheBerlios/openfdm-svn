/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TaskInfo_H
#define OpenFDM_TaskInfo_H

#include "SampleTime.h"

namespace OpenFDM {

class TaskInfo {
public:
  TaskInfo(void) :
    mSliceSize(0),
    mTime(0),
    mNumBasicSteps(0)
  {}

  void setTime(real_type t)
  { mTime = t; }
  real_type getTime(void) const
  { return mTime; }
  
  void addSampleTime(const SampleTime& sampleTime)
  { mSampleTimeSet.addSampleTime(sampleTime); }

  void clear(void)
  { mSampleTimeSet.clear(); }

  const SampleTimeSet& getSampleTimeSet(void) const
  { return mSampleTimeSet; }

  real_type getSliceSize(void) const
  { return mSliceSize; }
  void setSliceSize(real_type sliceSize)
  { mSliceSize = sliceSize; }

  unsigned getNumBasicSteps(void) const
  { return mNumBasicSteps; }
  void setNumBasicSteps(unsigned numBasicSteps)
  { mNumBasicSteps = numBasicSteps; }

private:
  SampleTimeSet mSampleTimeSet;
  real_type mSliceSize;
  real_type mTime;
  unsigned mNumBasicSteps;
};

} // namespace OpenFDM

#endif
