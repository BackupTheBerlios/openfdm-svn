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

class RealPortInterface;
class MatrixPortInterface;

class PortInterface : public Referenced {
public:
  virtual ~PortInterface(void) {}
  virtual RealPortInterface* toRealPortInterface(void) { return 0; }
  virtual MatrixPortInterface* toMatrixPortInterface(void) { return 0; }

  virtual bool isConnected(void) const = 0;
  virtual void evaluate(void) = 0;
};

class RealPortInterface : public PortInterface {
public:
  RealPortInterface(unsigned m = 1, unsigned n = 1) : mValue(m, n) {}
  virtual RealPortInterface* toRealPortInterface(void)
  {
    if (Size(1, 1) == size(mValue))
      return this;
    else
      return 0;
  }
  // FIXME, move evaluate into seperate method
  real_type getRealValue(void)
  { evaluate(); return mValue(1, 1); }
protected:
  Matrix mValue;
};

class MatrixPortInterface : public RealPortInterface {
public:
  virtual MatrixPortInterface* toMatrixPortInterface(void) { return this; }
  // FIXME, move evaluate into seperate method
  const Matrix& getMatrixValue(void)
  { evaluate(); return mValue; }
};


/// FIXME adapter to be somehow backwards compatible
/// Should vanish
class PropertyPortInterface : public MatrixPortInterface {
public:
  PropertyPortInterface(const Property& property) : mProperty(property)
  { }
  virtual void evaluate(void)
  {
    if (mProperty.isRealProperty()) {
      RealProperty rp = mProperty.toRealProperty();
      mValue.resize(1, 1);
      mValue(1, 1) = rp.getValue();
    } else if (mProperty.isMatrixProperty()) {
      MatrixProperty mp = mProperty.toMatrixProperty();
      mValue = mp.getValue();
    }
  }
  virtual bool isConnected(void) const
  { return mProperty.isValid(); }
private:
  mutable Property mProperty;
};
/// Just a getter used for now
template<typename M>
class RealGetterPortInterface : public MatrixPortInterface {
public:
  typedef const real_type& (M::*Getter) () const;
  RealGetterPortInterface(M* sourceModel, Getter getter) :
    mSourceModel(sourceModel), mGetter(getter)
  { }
  virtual void evaluate(void)
  { mValue(1, 1) = (mSourceModel->*mGetter)(); }
  virtual bool isConnected(void) const
  { return mSourceModel && mGetter; }
private:
  managed_ptr<M> mSourceModel;
  Getter mGetter;
};
template<typename M>
class MatrixGetterPortInterface : public MatrixPortInterface {
public:
  typedef const Matrix& (M::*Getter) () const;
  MatrixGetterPortInterface(M* sourceModel, Getter getter) :
    mSourceModel(sourceModel), mGetter(getter)
  { }
  virtual void evaluate(void)
  { mValue = (mSourceModel->*mGetter)(); }
  virtual bool isConnected(void) const
  { return mSourceModel && mGetter; }
private:
  managed_ptr<M> mSourceModel;
  Getter mGetter;
};

class RealPortHandle {
public:
  RealPortHandle(RealPortInterface* realPortInterface) :
    mRealPortInterface(realPortInterface)
  { }
  real_type getRealValue(void)
  { return mRealPortInterface->getRealValue(); }
  bool isConnected(void) const
  { return mRealPortInterface && mRealPortInterface->isConnected(); }
private:
  shared_ptr<RealPortInterface> mRealPortInterface;
};

class MatrixPortHandle {
public:
  MatrixPortHandle(MatrixPortInterface* matrixPortInterface) :
    mMatrixPortInterface(matrixPortInterface)
  { }
  const Matrix& getMatrixValue(void)
  { return mMatrixPortInterface->getMatrixValue(); }
  bool isConnected(void) const
  { return mMatrixPortInterface && mMatrixPortInterface->isConnected(); }
private:
  shared_ptr<MatrixPortInterface> mMatrixPortInterface;
};

// should vanish, just an adaptor for smoother migration
class RealPortExpression : public PropertyImpl<real_type> {
public:
  RealPortExpression(const RealPortHandle& rph) :
    mRealPortHandle(rph)
  { }
  real_type getValue(void) const
  { return mRealPortHandle.getRealValue(); }
  void setValue(const real_type&)
  {  }
  bool isValid(void) const { return mRealPortHandle.isConnected(); }
  const Object* getObject(void) const { return 0; }
  Object* getObject(void) { return 0; }
private:
  mutable RealPortHandle mRealPortHandle;
};

/// Class for an input or output port of a Model.
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
    setPortInterface(new PropertyPortInterface(property));
  }
  void setPortInterface(PortInterface* portInterface)
  {
    mPortInterface = portInterface;
    std::vector<shared_ptr<Port> >::iterator it;
    for (it = mChainPorts.begin(); it != mChainPorts.end(); ++it) {
      (*it)->setPortInterface(mPortInterface);
    }
  }

  /// Just use the Properties for now. In this phase it might be a good idea.
  Property getProperty(void) const
  { return Property(new RealPortExpression(((Port*)(this))->toRealPortHandle())); }


  /// returns true if this port has a source port connected to it
  bool isConnected() const
  { return mPortInterface && mPortInterface->isConnected(); }

  /// returns true if the source port sourcePort is the value source for the
  /// current port
  bool isConnectedTo(const Port* sourcePort) const
  {
    const Port* port = mSourcePort;
    while (port) {
      if (sourcePort == port)
        return true;
      port = port->mSourcePort;
    }

    return false;
  }

  RealPortHandle toRealPortHandle(void)
  { return RealPortHandle(mPortInterface->toRealPortInterface()); }
  MatrixPortHandle toMatrixPortHandle(void)
  { return MatrixPortHandle(mPortInterface->toMatrixPortInterface()); }

  /// Retrieve the value of this port
  /// Note that we don't need a setValue method since we attach a getter of a
  /// Model to a port.

  /// This might be the place where it is possible to implement
  /// TaskInfo dependent output ports ...
  /// Hmm, may be we should otoh 'dirty' some getters?
  /// Generic thing. Don't use if you don't have to
  Variant getValue(void)
  {
    if (mPortInterface) {
      RealPortInterface* realPortInterface
        = mPortInterface->toRealPortInterface();
      if (realPortInterface)
        return Variant(realPortInterface->getRealValue());
      MatrixPortInterface* matrixPortInterface
        = mPortInterface->toMatrixPortInterface();
      if (matrixPortInterface)
        return Variant(matrixPortInterface->getMatrixValue());
    }
    return Variant();
  }

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
    setPortInterface(sourcePort->mPortInterface);
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
    setPortInterface(0);
  }

  /// Just disconnect from whoever we are connected to
  void disconnect(void)
  { disconnect(mSourcePort); }

private:
  /// For now the untyped input port
  /// On Model::init() it is expected to be specialized
  /// to a typed port handle
  shared_ptr<PortInterface> mPortInterface;
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
private:
  // Sets the parent model.
  // That is the one which is informed if the number of states changes.
  void setParent(ModelGroup* modelGroup);
  const ModelGroup* getParent(void) const { return mParentModel; }
  ModelGroup* getParent(void) { return mParentModel; }
  void adjustNumContinousStates(unsigned newCount, unsigned oldCount);
  void adjustNumDiscreteStates(unsigned newCount, unsigned oldCount);

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
