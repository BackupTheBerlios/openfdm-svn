/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Port_H
#define OpenFDM_Port_H

#include <string>
#include <vector>
#include <algorithm>

#include "LogStream.h"
#include "Object.h"
#include "Property.h"
#include "Variant.h"

namespace OpenFDM {

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
  RealPortHandle(void)
  { }
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
  MatrixPortHandle(void)
  { }
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
  virtual ~Port(void);

  /// Just use the Properties for now. In this phase it might be a good idea.
  void setProperty(const Property& property)
  { setPortInterface(new PropertyPortInterface(property)); }
  void setPortInterface(PortInterface* portInterface);

  /// Just use the Properties for now. In this phase it might be a good idea.
  Property getProperty(void) const
  { return Property(new RealPortExpression(((Port*)(this))->toRealPortHandle())); }


  /// returns true if this port has a source port connected to it
  bool isConnected() const
  { return mPortInterface && mPortInterface->isConnected(); }

  /// returns true if the source port sourcePort is the value source for the
  /// current port
  bool isConnectedTo(const Port* sourcePort) const;

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
  Variant getValue(void);

  /// Connect this port to the given source port
  void connect(Port* sourcePort);

  /// Disconnect this port from the given source port
  void disconnect(Port* sourcePort);

  /// Just disconnect from whoever we are connected to
  void disconnect(void);

private:
  /// For now the untyped input port
  /// On Model::init() it is expected to be specialized
  /// to a typed port handle
  shared_ptr<PortInterface> mPortInterface;
  /// The list of readers for this port
  std::vector<shared_ptr<Port> > mChainPorts;
  /// The source of the current port connection
  managed_ptr<Port> mSourcePort;
};

} // namespace OpenFDM

#endif
