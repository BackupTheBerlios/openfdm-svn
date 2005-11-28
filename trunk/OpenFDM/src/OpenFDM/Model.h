/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
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

namespace OpenFDM {

class ModelGroup;
class Input;
class Output;

/// Class for an inout or output port of a Model.
/// Ports can be connected together. This means in effect that the reader
/// gains access to value at the source model.
/// Additional information must be carried through that class.
/// ...
class Port :
    public Object {
public:
  virtual ~Port(void) {}

  /// Returns the systems name.
  const std::string& getName(void) const
  { return mName; }
  void setName(const std::string& name)
  { mName = name; }

  /// Just use the Properties for now. In this phase it might be a good idea.
  void setProperty(const Property& property)
  {
    mProperty = property;
    std::vector<shared_ptr<Port> >::iterator it;
    for (it = mChainPorts.begin(); it != mChainPorts.end(); ++it) {
      (*it)->setProperty(property);
    }
  }
  /// Just use the Properties for now. In this phase it might be a good idea.
  const Property& getProperty(void) const
  { return mProperty; }
  Property& getProperty(void)
  { return mProperty; }

  /// returns true if this port has a source port connected to it
  bool isConnected() const
  { return mProperty.isValid(); }

  /// Retrieve the value of this port
  /// Note that we don't need a setValue method since we attach a getter of a
  /// Model to a port.

  /// This might be the place where it is possible to implement
  /// TaskInfo dependent output ports ...
  /// Hmm, may be we should otoh 'dirty' some getters?
  Variant getValue(void) const
  { return mProperty.getValue(); }

  /// Connect this port to the given source port
  void connect(Port* sourcePort)
  {
    if (!sourcePort) {
      Log(Model, Warning) << "Part argument to Port::connect is zero!"
        "Ignoring!" << endl;
      return;
    }
    // disconnect from other source if we are already connected
    // FIXME: should this be explicit ??
    if (mSourcePort) {
      Log(Model, Warning) << "Connecting already connected port!"
        "Disconnecting from old port now." << endl;
      disconnect(mSourcePort);
    }

    // If we have a source port, propagate its context.
    setProperty(sourcePort->getProperty());
    sourcePort->mChainPorts.push_back(this);
    mSourcePort = sourcePort;
  }

  /// Disconnect this port from the given source port
  void disconnect(Port* sourcePort)
  {
    if (sourcePort != mSourcePort) {
      Log(Model, Error) << "Try to disconnect from source port we are not "
        "currently connected to!" << endl;
      return;
    }

    if (!mSourcePort)
      return;

    // Remove ourselves from the consumer list of the sourcePort to
    // disconnect us from
    std::vector<shared_ptr<Port> >::iterator it, beginPort, endPort;
    beginPort = sourcePort->mChainPorts.begin();
    endPort = sourcePort->mChainPorts.end();
    it = std::find(beginPort, endPort, this);
    if (it != endPort)
      sourcePort->mChainPorts.erase(it);

    // Reset our source
    mSourcePort = 0;

    // Invalidate all our listeners
    setProperty(Property());
  }

  /// Just disconnect from whoever we are connected to
  void disconnect(void)
  { disconnect(mSourcePort); }

private:
  mutable/*FIXME*/ Property mProperty;
  /// The list of readers for this port
  std::vector<shared_ptr<Port> > mChainPorts;
  /// The source of the current port connection
  managed_ptr<Port> mSourcePort;
  /// The current ports name FIXME does this belong here?
  std::string mName;
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

  virtual void setState(const Vector& state, unsigned offset);
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

  std::string mName;
  managed_ptr<ModelGroup> mParentModel;
  unsigned mNumContinousStates;
  unsigned mNumDiscreteStates;
  bool mDirectFeedThrough;
  SampleTimeSet mSampleTimeSet;
  std::vector<shared_ptr<Port> > mInputPorts;
  std::vector<shared_ptr<Port> > mOutputPorts;

  // FIXME
  friend class ModelGroup;
};

} // namespace OpenFDM

#endif
