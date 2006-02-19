/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Port_H
#define OpenFDM_Port_H

#include <string>
#include <vector>
#include <algorithm>

#include "LogStream.h"
#include "Object.h"
#include "WeakPtr.h"
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
    evaluate();
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
  WeakPtr<M> mSourceModel;
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
  WeakPtr<M> mSourceModel;
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
  SharedPtr<RealPortInterface> mRealPortInterface;
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
  SharedPtr<MatrixPortInterface> mMatrixPortInterface;
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

  void setPortInterface(PortInterface* portInterface);

  /// returns true if this port has a source port connected to it
  bool isConnected() const
  { return mPortInterface && mPortInterface->isConnected(); }

  /// returns true if the source port sourcePort is the value source for the
  /// current port
  bool isConnectedTo(const Port* sourcePort) const;

  /// returns true if the source port sourcePort is the value source for the
  /// current port
  bool hasSameSource(const Port* otherPort) const;

  RealPortHandle toRealPortHandle(void)
  {
    if (mPortInterface)
      return RealPortHandle(mPortInterface->toRealPortInterface());
    else
      return RealPortHandle(0);
  }
  MatrixPortHandle toMatrixPortHandle(void)
  {
    if (mPortInterface)
      return MatrixPortHandle(mPortInterface->toMatrixPortInterface());
    else
      return MatrixPortHandle(0);
  }

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
  SharedPtr<PortInterface> mPortInterface;
  /// The list of readers for this port
  std::vector<SharedPtr<Port> > mChainPorts;
  /// The source of the current port connection
  WeakPtr<Port> mSourcePort;
};

} // namespace OpenFDM

#endif
