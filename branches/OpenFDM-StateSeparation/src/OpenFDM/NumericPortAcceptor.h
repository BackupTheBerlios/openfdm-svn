/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NumericPortAcceptor_H
#define OpenFDM_NumericPortAcceptor_H

#include "PortAcceptor.h"

namespace OpenFDM {

class RealPortInterface;
class MatrixPortInterface;

class PortInterface : public Referenced {
public:
  virtual ~PortInterface(void) {}
  virtual RealPortInterface* toRealPortInterface(void) { return 0; }
  virtual MatrixPortInterface* toMatrixPortInterface(void) { return 0; }

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
  { evaluate(); return mValue(0, 0); }
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
  { mValue(0, 0) = (mSourceModel.lock()->*mGetter)(); }
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
  { mValue = (mSourceModel.lock()->*mGetter)(); }
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
  { return mRealPortInterface; }
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
  { return mMatrixPortInterface; }
private:
  SharedPtr<MatrixPortInterface> mMatrixPortInterface;
};

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

class NumericPortAcceptor :
    public PortAcceptor {
public:
  NumericPortAcceptor(Node* node) : PortAcceptor(node) {}

  virtual Port::ConnectResult addConnection(Connection* connection)
  {
    Port::ConnectResult result = PortAcceptor::addConnection(connection);
    if (result != Port::Success)
      return result;
    mConnection = connection;
    return Port::Success;
  }
  virtual bool removeConnection(Connection* connection)
  {
    if (!connection || connection != mConnection)
      return false;
    setPortInterface(0);
    PortAcceptor::removeConnection(connection);
    mConnection = 0;
    return true;
  }
  virtual void removeAllConnections()
  {
    if (mConnection)
      removeConnection(mConnection);
  }

  virtual Port::ConnectResult connect(PortProvider* port)
  { return port->provide(this); }
  virtual bool disconnect(PortProvider* port)
  { return port->unprovide(this); }




  PortInterface* getPortInterface() const
  { return mPortInterface; }

  virtual Port::ConnectResult setPortInterface(PortInterface* portInterface)
  {
    mPortInterface = portInterface;
    return Port::Success;
  }

  /// Legacy ones
  RealPortHandle toRealPortHandle(void)
  {
    if (getPortInterface())
      return RealPortHandle(getPortInterface()->toRealPortInterface());
    else
      return RealPortHandle(0);
  }
  MatrixPortHandle toMatrixPortHandle(void)
  {
    if (getPortInterface())
      return MatrixPortHandle(getPortInterface()->toMatrixPortInterface());
    else
      return MatrixPortHandle(0);
  }

private:
  SharedPtr<PortInterface> mPortInterface;
  SharedPtr<Connection> mConnection;
};

} // namespace OpenFDM

#endif
