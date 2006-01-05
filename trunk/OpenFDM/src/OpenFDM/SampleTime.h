/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SampleTime_H
#define OpenFDM_SampleTime_H

#include <iosfwd>
#include <vector>

#include "Assert.h"

namespace OpenFDM {

/// The discrete sample time for this model
/// There are special meanings encoded into that value:
/// - positive real number, this is the discrete sample time itself
/// - zero, continous updates
/// - otherwise, inherited from its parent model group
/// FIXME: need to have a sample time slot which is only evaluated if the
/// simulation returns to the user. Just to avoid additional effort for
/// animations
class SampleTime {
public:
  /// Default constructor, defaults to discrete sample time
  SampleTime(void) : mSampleTime(0) {}
  /// Constructor with given sample time
  SampleTime(real_type sampleTime) : mSampleTime(sampleTime) {}
  /// Returns true if the sample time is a continous sample time
  bool isContinous(void) const
  { return mSampleTime == 0; }
  /// Returns true if the sample time is discrete
  bool isDiscrete(void) const
  { return 0 < mSampleTime; }
  /// Returns true if the sample time is just inheritted
  /// FIXME: empty sample time list??
  bool isInherited(void) const
  { return mSampleTime == -1; }
  /// Returns true if the task is a per timestep task
  bool isPerTimestep(void) const
  { return mSampleTime == -2; }
  /// Returns the actual sample time
  real_type getSampleTime(void) const
  { return mSampleTime; }

  /// Returns true if th sample time is valid
  bool isValid(void) const
  { return isContinous() || isDiscrete() || isInherited() || isPerTimestep(); }

  bool operator==(const SampleTime& st) const
  { return mSampleTime == st.mSampleTime; }
  bool operator!=(const SampleTime& st) const
  { return mSampleTime != st.mSampleTime; }

  static const SampleTime PerTimestep;
  static const SampleTime Inherited;
  static const SampleTime Continous;

private:
  real_type mSampleTime;
};

class SampleTimeSet {
  typedef std::vector<SampleTime> SampleTimeData;

public:
  typedef SampleTimeData::iterator iterator;
  typedef SampleTimeData::const_iterator const_iterator;

  iterator begin(void)
  { return mSampleTimes.begin(); }
  iterator end(void)
  { return mSampleTimes.end(); }

  const_iterator begin(void) const
  { return mSampleTimes.begin(); }
  const_iterator end(void) const
  { return mSampleTimes.end(); }

  bool empty(void) const
  { return mSampleTimes.empty(); }

  bool addSampleTime(const SampleTime& sampleTime)
  {
    OpenFDMAssert(sampleTime.isValid());
    if (!sampleTime.isValid())
      return false;
    SampleTimeData::iterator it;
    for (it = mSampleTimes.begin(); it != mSampleTimes.end(); ++it) {
      // If the sample time is already included don't include twice
      if (it->getSampleTime() == sampleTime.getSampleTime())
        return true;
      // If we found a sample time bigger than the new one, insert the
      // new one before it
      if (sampleTime.getSampleTime() < it->getSampleTime())
        break;
    }
    // insert before it
    mSampleTimes.insert(it, sampleTime);
    return true;
  }
  bool removeSampleTime(const SampleTime& sampleTime)
  {
    OpenFDMAssert(sampleTime.isValid());
    if (!sampleTime.isValid())
      return false;
    SampleTimeData::iterator it;
    for (it = mSampleTimes.begin(); it != mSampleTimes.end(); ++it) {
      // If the sample time is already included don't include twice
      if (it->getSampleTime() == sampleTime.getSampleTime()) {
        mSampleTimes.erase(it);
        return true;
      }
    }
    return false;
  }

  void clear(void)
  { mSampleTimes.resize(0); }

private:
  /// Contains an ascending sorted vector of sample times belonging
  /// to the current set
  // FIXME May be std::set is a good alternative?
  SampleTimeData mSampleTimes;
};

template<typename char_type, typename traits_type> 
inline
std::basic_ostream<char_type, traits_type>&
operator<<(std::basic_ostream<char_type, traits_type>& stream,
           const SampleTimeSet& sts)
{
  stream << "{ ";
  SampleTimeSet::const_iterator it;
  for (it = sts.begin(); it != sts.end(); ++it)
    stream << it->getSampleTime() << " ";
  stream << "}";
  return stream;
}

inline
bool
nonZeroIntersection(const SampleTimeSet& set1, const SampleTime& sampleTime)
{
  SampleTimeSet::const_iterator it;
  for (it = set1.begin(); it != set1.end(); ++it) {
    if (*it == sampleTime)
      return true;
  }
  return false;
}

inline
bool
nonZeroIntersection(const SampleTime& sampleTime, const SampleTimeSet& set1)
{
  return nonZeroIntersection(set1, sampleTime);
}

inline
bool
nonZeroIntersection(const SampleTimeSet& set1, const SampleTimeSet& set2)
{
  SampleTimeSet::const_iterator it1 = set1.begin();
  SampleTimeSet::const_iterator it2 = set2.begin();
  while (it1 != set1.end() && it2 != set2.end()) {
    if (it1->getSampleTime() == it2->getSampleTime())
      return true;
    if (it1->getSampleTime() < it2->getSampleTime())
      ++it1;
    else
      ++it2;
  }
  return false;
}

} // namespace OpenFDM

#endif
