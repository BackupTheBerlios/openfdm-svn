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
#include "Property.h"
#include "Variant.h"
#include "Vector.h"
#include "SampleTime.h"
#include "TaskInfo.h"
#include "StateStream.h"
#include "Port.h"

namespace OpenFDM {

class ModelGroup;
class MultiBodySystem;
class Environment;

class Input;
class Output;
class Interact;
class Joint;
class MobileRootJoint;

class ModelVisitor;

class Model : public Object {
  OPENFDM_OBJECT(Model, Object);
public:
  Model(const std::string& name);
  virtual ~Model(void);

  /// Double dispatch helper for the multibody system visitor
  virtual void accept(ModelVisitor& visitor);
  /// Double dispatch helper for the multibody system visitor
//   virtual void accept(ConstModelVisitor& visitor) const;

  /// Hmm ...
  void ascend(ModelVisitor& visitor)
  { if (mParentModel) mParentModel->accept(visitor); }

  virtual const ModelGroup* toModelGroup(void) const;
  virtual ModelGroup* toModelGroup(void);

  virtual const Input* toInput(void) const;
  virtual Input* toInput(void);

  virtual const Output* toOutput(void) const;
  virtual Output* toOutput(void);

  virtual const Interact* toInteract(void) const;
  virtual Interact* toInteract(void);

  virtual const Joint* toJoint(void) const;
  virtual Joint* toJoint(void);

  virtual const MobileRootJoint* toMobileRootJoint(void) const;
  virtual MobileRootJoint* toMobileRootJoint(void);

  /// Called on each system initialization.
  virtual bool init(void);
  /// Called when the outputs need to be prepared for the next step.
  /// Note that this is called *before* update() is called.
  virtual void output(const TaskInfo& taskInfo);
  /// Called whenever discrete states need to be updated.
  virtual void update(const TaskInfo& taskInfo);

  virtual void setState(const StateStream& state);
  virtual void getState(StateStream& state) const;
  virtual void getStateDeriv(StateStream& stateDeriv);

  virtual void setDiscreteState(const StateStream& state);
  virtual void getDiscreteState(StateStream& state) const;

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

  /// Returns true if it needs to know the accelerations ...
  /// FIXME: do that in a different way ...
  bool getMultiBodyAcceleration(void) const
  { return mMultiBodyAcceleration; }

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
  Port* getInputPort(const std::string& name);
  Port* getInputPort(unsigned i)
  {
    OpenFDMAssert(i < mInputPorts.size());
    return mInputPorts[i];
  }

  unsigned getNumOutputPorts(void) const
  { return mOutputPorts.size(); }

  Port* getOutputPort(unsigned i);
  Port* getOutputPort(const std::string& name);
  const std::string& getOutputPortName(unsigned i) const;

  std::string getPathString(void) /* FIXME const*/;

  const Model* getParent(void) const { return mParentModel; }
  Model* getParent(void) { return mParentModel; }

  /// FIXME: have function returning the input ports a given output
  /// port depends on

protected:
  void setNumContinousStates(unsigned numContinousStates);
  void setNumDiscreteStates(unsigned numDiscreteStates);
  void setDirectFeedThrough(bool directFeedThrough)
  { mDirectFeedThrough = directFeedThrough; }
  void setMultiBodyAcceleration(bool multiBodyAcceleration)
  { mMultiBodyAcceleration = multiBodyAcceleration; }

  /// Sets the number of input properties.
  void setNumInputPorts(unsigned num);

  /// Sets the name of the i-th input property.
  void setInputPortName(unsigned i, const std::string& name);

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

  virtual Environment* getEnvironment(void) const;

private:
  // Sets the parent model.
  // That is the one which is informed if the number of states changes.
  void setParent(Model* model);

  /// FIXME: use visitor for that
  void adjustNumContinousStates(unsigned newCount, unsigned oldCount);
  void adjustNumDiscreteStates(unsigned newCount, unsigned oldCount);

  WeakPtr<Model> mParentModel;
  unsigned mNumContinousStates;
  unsigned mNumDiscreteStates;
  bool mDirectFeedThrough;
  // FIXME, at the moment used to state that this model must be scheduled
  // past all joint interacts
  bool mMultiBodyAcceleration;
  SampleTimeSet mSampleTimeSet;
  std::vector<SharedPtr<Port> > mInputPorts;
  std::vector<SharedPtr<Port> > mOutputPorts;

  // FIXME
  friend class ModelGroup;
  friend class MultiBodySystem;
};

} // namespace OpenFDM

#endif
