/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Model_H
#define OpenFDM_Model_H

#include <string>
#include <vector>
#include <algorithm>

#include "Assert.h"
#include "Object.h"
#include "Property.h"
#include "Vector.h"

namespace OpenFDM {

class ModelGroup;
class Input;
class Output;

/// The discrete sample time for this model
/// There are special meanings encoded into that value:
/// - positive real number, this is the discrete sample time itself
/// - zero, continous updates
/// - otherwise, inherited from its parent model group
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

class TaskInfo {
public:
  TaskInfo(void) :
    mSliceSize(0),
    mNumBasicSteps(0)
  {}
  
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
  unsigned mNumBasicSteps;
};

class Model
  : public Object {
public:
  Model(const std::string& name);
  virtual ~Model(void);

  virtual const ModelGroup* toModelGroup(void) const;
  virtual ModelGroup* toModelGroup(void);

  virtual const Input* toInput(void) const;
  virtual Input* toInput(void);

  virtual const Output* toOutput(void) const;
  virtual Output* toOutput(void);

  /// Called on each system initialization.
  virtual bool init(void);
  /// Called when the outputs need to be prepared for the next step.
  /// Note that this is called *before* update() is called.
  virtual void output(const TaskInfo& taskInfo);
  /// Called whenever discrete states need to be updated.
  virtual void update(const TaskInfo& taskInfo);

  virtual void setState(real_type t, const Vector& state, unsigned offset);
  virtual void getState(Vector& state, unsigned offset) const;
  virtual void getStateDeriv(Vector& stateDeriv, unsigned offset);

  virtual void setDiscreteState(const Vector& state, unsigned offset);
  virtual void getDiscreteState(Vector& state, unsigned offset) const;

  /// FIXME Hmm, may be different ...
  /// May move into System ...
  void evalFunction(real_type t, const Vector& v, Vector& out);
  /// Compute the jacobian
  /// The default implementation computes a numeric approximation by finite
  /// differences
  void evalJacobian(real_type t, const Vector& state, Matrix& jac);

  /// Return the number of continous states
  unsigned getNumContinousStates(void) const
  { return mNumContinousStates; }
  /// Return the number of discrete states
  unsigned getNumDiscreteStates(void) const
  { return mNumDiscreteStates; }
  /// Return if the outputs containe a direct dependency on an input
  bool getDirectFeedThrough(void) const
  { return mDirectFeedThrough; }

  bool addSampleTime(const SampleTime& sampleTime)
  { return mSampleTimeSet.addSampleTime(sampleTime); }
  bool removeSampleTime(const SampleTime& sampleTime)
  { return mSampleTimeSet.removeSampleTime(sampleTime); }
  const SampleTimeSet& getSampleTimeSet(void) const
  { return mSampleTimeSet; }
  

  /// Returns the systems name.
  const std::string& getName(void) const
  { return mName; }
  void setName(const std::string& name)
  { mName = name; }

  /// Returns the number of input properties.
  unsigned getNumInputPorts(void) const
  { return mInputPorts.size(); }

  /// Returns the name of the i-th input property.
  const std::string& getInputPortName(unsigned i) const;

  /// Sets the i-th input property.
  bool setInputPort(unsigned i, const Property& prop);

  /// Sets the input with the given name property.
  bool setInputPort(const std::string& name, const Property& prop);

  const Property& getInputPort(const std::string& name) const;
  Property& getInputPort(const std::string& name);

  /// Returns the i-th input property.
  const Property& getInputPort(unsigned i) const
  {
    OpenFDMAssert(i < mInputPorts.size());
    return mInputPorts[i].property;
  }
  /// Returns the i-th input property.
  Property& getInputPort(unsigned i)
  {
    OpenFDMAssert(i < mInputPorts.size());
    return mInputPorts[i].property;
  }

  unsigned getNumOutputPorts(void) const
  { return mOutputPorts.size(); }

  Property getOutputPort(unsigned i) const;
  const std::string& getOutputPortName(unsigned i) const;

  /// Returns the systems output property with the given name.
  Property getOutputPort(const std::string& name) const;

  bool dependsDirectOn(const Model* const model) const;

protected:
  void setNumContinousStates(unsigned numContinousStates);
  void setNumDiscreteStates(unsigned numDiscreteStates);
  void setDirectFeedThrough(bool directFeedThrough)
  { mDirectFeedThrough = directFeedThrough; }

  /// Sets the number of input properties.
  void setNumInputPorts(unsigned num);

  /// Sets the name of the i-th input property.
  void setInputPortName(unsigned i, const std::string& name);

  /// Gets called whenever an input property is changed.
  /// Lets a Model implementation catch up changes to the model.
  virtual void inputPortChanged(unsigned i);

  /// Sets the number of output properties.
  void setNumOutputPorts(unsigned num);

  /// Sets the name of the i-th output property.
  void setOutputPort(unsigned i, const std::string& name,
                     const Property& prop);

private:
  // Sets the parent model.
  // That is the one which is informed if the number of states changes.
  void setParent(ModelGroup* modelGroup);
  const ModelGroup* getParent(void) const { return mParentModel; }
  ModelGroup* getParent(void) { return mParentModel; }
  void adjustNumContinousStates(unsigned newCount, unsigned oldCount);
  void adjustNumDiscreteStates(unsigned newCount, unsigned oldCount);

  /// Holds the single input property.
  struct Port {
    Property    property;
    std::string name;
  };
  std::vector<Port> mInputPorts;
  managed_ptr<ModelGroup> mParentModel;
  std::string mName;
  unsigned mNumContinousStates;
  unsigned mNumDiscreteStates;
  bool mDirectFeedThrough;
  std::vector<Port> mOutputPorts;
  SampleTimeSet mSampleTimeSet;

  // FIXME
  friend class ModelGroup;
};

} // namespace OpenFDM

#endif
