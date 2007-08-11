/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Model_H
#define OpenFDM_Model_H

#include <string>
#include <vector>
#include <algorithm>

#include "Assert.h"
#include "LogStream.h"
#include "Object.h"
#include "Variant.h"
#include "Vector.h"
#include "SampleTime.h"
#include "TaskInfo.h"
#include "StateStream.h"
#include "Connection.h"
#include "Port.h"
#include "NumericPortProvider.h"
#include "NumericPortAcceptor.h"

namespace OpenFDM {

class ModelGroup;
class Environment;
class GroupInput;
class GroupOutput;

class Input;
class Output;
class Interact;
class Joint;
class MobileRootJoint;

class ModelVisitor;
class TaskInfo;

class Model : public Object {
  OPENFDM_OBJECT(Model, Object);
public:
  enum DisableMode {
    /// If disabled, the models output/state is just held.
    /// On reenable, the model just continues to work
    Hold,
    /// If disabled, the models output/state is just held.
    /// On reenable, the model is initialized
    HoldReset,
    /// If disabled, the models output/state is initialized
    ResetHold
  };

  typedef std::list<SharedPtr<ModelGroup> > Path;

  Model(const std::string& name);
  virtual ~Model(void);

  /// Double dispatch helper for the system visitor
  virtual void accept(ModelVisitor& visitor);
  /// Double dispatch helper for the system visitor
//   virtual void accept(ConstModelVisitor& visitor) const;

  /// Hmm ...
  void ascend(ModelVisitor& visitor);

  virtual const ModelGroup* toModelGroup(void) const;
  virtual ModelGroup* toModelGroup(void);

  virtual const Input* toInput(void) const;
  virtual Input* toInput(void);

  virtual const Output* toOutput(void) const;
  virtual Output* toOutput(void);

  virtual const Interact* toInteract(void) const;
  virtual Interact* toInteract(void);

  /// Called on each system initialization.
  virtual bool init(void);
  /// Called when the outputs need to be prepared for the next step.
  /// Note that this is called *before* update() is called.
  virtual void output(const TaskInfo& taskInfo /*const TaskSet& ??*/);
  /// Called whenever discrete states need to be updated.
  virtual void update(const TaskInfo& taskInfo /*const Task& ??*/);

  /// Convinience functions may make the virtuals protected ...
  void outputIfEnabled(const TaskInfo& taskInfo)
  {
    /// FIXME: should that only be checked for discrete tasks???
    /// May be with a special list in System to know which ones need to recheck
    /// the enable ports ???
    setEnabled(mNextEnabled);
    if (mEnabled)
      output(taskInfo);
  }
  void updateIfEnabled(const TaskInfo& taskInfo)
  {
    if (mEnabled)
      update(taskInfo);
    if (mEnablePortHandle.isConnected()) {
      /// FIXME bool!!!!
      mNextEnabled = 0.5 < fabs(mEnablePortHandle.getRealValue());
    }
  }

  virtual void setState(const StateStream& state);
  virtual void getState(StateStream& state) const;
  virtual void getStateDeriv(StateStream& stateDeriv);

  virtual void setDiscreteState(const StateStream& state);
  virtual void getDiscreteState(StateStream& state) const;

  /// Must return true if the model given in the argument must be scheduled
  /// before this one because of input data dependencies
  virtual bool dependsDirectOn(Model* model);

  /// Return the number of continous states
  unsigned getNumContinousStates(void) const
  { return mNumContinousStates; }
  /// Return the number of discrete states
  unsigned getNumDiscreteStates(void) const
  { return mNumDiscreteStates; }
  /// Return if the outputs containe a direct dependency on an input
  bool getDirectFeedThrough(void) const
  { return mDirectFeedThrough; }
  /// Return if the outputs containe a direct dependency on an input
  bool getEnabled(void) const
  { return mEnabled; }
  void setEnabled(bool enabled)
  { if (mEnabled != enabled) setEnabledUnconditional(enabled); }
  DisableMode getDisableMode(void) const
  { return mDisableMode; }
  void setDisableMode(DisableMode disableMode)
  { mDisableMode = disableMode; }

  bool addSampleTime(const SampleTime& sampleTime)
  { return mSampleTimeSet.addSampleTime(sampleTime); }
  bool removeSampleTime(const SampleTime& sampleTime)
  { return mSampleTimeSet.removeSampleTime(sampleTime); }
  const SampleTimeSet& getSampleTimeSet(void) const
  { return mSampleTimeSet; }
  

  /// Returns the number of input properties.
  unsigned getNumInputPorts(void) const
  { return mInputPorts.size(); }

  /// Returns the name of the i-th input property.
  const std::string& getInputPortName(unsigned i) const;

  /// Sets the i-th input property.
  NumericPortAcceptor* getInputPort(const std::string& name);
  NumericPortAcceptor* getInputPort(unsigned i)
  {
    OpenFDMAssert(i < mInputPorts.size());
    return mInputPorts[i];
  }
  NumericPortAcceptor* getEnablePort(void)
  { return mEnablePort; }

  unsigned getNumOutputPorts(void) const
  { return mOutputPorts.size(); }

  NumericPortProvider* getOutputPort(unsigned i);
  NumericPortProvider* getOutputPort(const std::string& name);
  const std::string& getOutputPortName(unsigned i) const;

  std::string getPathString(void) /* FIXME const*/;
  Path getPath() /* FIXME const*/;

  const ModelGroup* getParent(void) const;
  ModelGroup* getParent(void);

protected:
  void setNumContinousStates(unsigned numContinousStates);
  void setNumDiscreteStates(unsigned numDiscreteStates);
  void setDirectFeedThrough(bool directFeedThrough)
  { mDirectFeedThrough = directFeedThrough; }

  /// Sets the number of input properties.
  void setNumInputPorts(unsigned num);

  /// Sets the name of the i-th input property.
  void setInputPortName(unsigned i, const std::string& name);
  void addInputPort(NumericPortAcceptor* port)
  {
    unsigned num = getNumInputPorts();
    setNumInputPorts(num+1);
    mInputPorts[num] = port;
  }
  void removeInputPort(NumericPortAcceptor* port)
  {
    InputPortVector::iterator i = std::find(mInputPorts.begin(),
                                            mInputPorts.end(), port);
    if (i == mInputPorts.end()) {
      Log(Model,Error) << "Trying to remove foreign port" << endl;
      return;
    }
    (*i)->invalidate();
    mInputPorts.erase(i);
  }

  /// Sets the number of output properties.
  void setNumOutputPorts(unsigned num);

  /// Sets the name of the i-th output property.
  void setOutputPort(unsigned i, const std::string& name,
                     PortInterface* portInterface);
  /// the real used interface
  template<typename M>
  void setOutputPort(unsigned i, const std::string& name, M* model,
                     const real_type& (M::*getter)(void) const)
  { setOutputPort(i, name, new RealGetterPortInterface<M>(model, getter)); }
  template<typename M>
  void setOutputPort(unsigned i, const std::string& name, M* model,
                     const Matrix& (M::*getter)(void) const)
  { setOutputPort(i, name, new MatrixGetterPortInterface<M>(model, getter)); }

  /// Convenience shortcuts
  void addOutputPort(NumericPortProvider* port)
  {
    unsigned num = getNumOutputPorts();
    setNumOutputPorts(num+1);
    mOutputPorts[num] = port;
  }
  void removeOutputPort(NumericPortProvider* port)
  {
    OutputPortVector::iterator i = std::find(mOutputPorts.begin(),
                                             mOutputPorts.end(), port);
    if (i == mOutputPorts.end()) {
      Log(Model,Error) << "Trying to remove foreign port" << endl;
      return;
    }
    (*i)->invalidate();
    mOutputPorts.erase(i);
  }

  void addOutputPort(const std::string& name, PortInterface* portInterface)
  {
    unsigned num = getNumOutputPorts();
    setNumOutputPorts(num+1);
    setOutputPort(num, name, portInterface);
  }
  template<typename M>
  void addOutputPort(const std::string& name, M* model,
                     const real_type& (M::*getter)(void) const)
  { addOutputPort(name, new RealGetterPortInterface<M>(model, getter)); }
  template<typename M>
  void addOutputPort(const std::string& name, M* model,
                     const Matrix& (M::*getter)(void) const)
  { addOutputPort(name, new MatrixGetterPortInterface<M>(model, getter)); }

  virtual void setEnvironment(Environment* environment);

// private:
  // Sets the parent model.
  // That is the one which is informed if the number of states changes.
  virtual void setParent(ModelGroup* model);
  friend class GroupInput;
  friend class GroupOutput;

private:
  void setEnabledUnconditional(bool enabled);

  WeakPtr<ModelGroup> mParentModel;
  unsigned mNumContinousStates;
  unsigned mNumDiscreteStates;
  bool mDirectFeedThrough;
  /// True if the Model is enabled
  bool mEnabled;
  bool mNextEnabled;
  SharedPtr<NumericPortAcceptor> mEnablePort;
  RealPortHandle mEnablePortHandle;
  DisableMode mDisableMode;
  SampleTimeSet mSampleTimeSet;

protected:
  typedef std::vector<SharedPtr<NumericPortAcceptor> > InputPortVector;
  typedef std::vector<SharedPtr<NumericPortProvider> > OutputPortVector;
  InputPortVector mInputPorts;
  OutputPortVector mOutputPorts;

  // FIXME
  friend class ModelGroup;
};

} // namespace OpenFDM

#endif
