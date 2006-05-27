/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Port_H
#define OpenFDM_Port_H

#include "Object.h"
#include "SharedPtr.h"
#include "WeakPtr.h"

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

class Model;
class Connection;

class Port :
    public Object {
public:
  enum ConnectResult {
    Success = 0,
    NoPort, // The given port is a zero pointer
    NoConnection, // The given Connection is a zero pointer
    Running, // The System is running, stop the System before manipulating it
    IsolatedModel, // Model does not belong to a parent Group
    DifferentGroups, // Model belong to different model groups
    StalePort, // Post still alive but originating model is dead
    AlreadyConnected, // This end of the connection is already connected
    IncompatiblePort, // The port is not compatible with the other end
    IncompatibelSize  // The port is not compatible with the other end
  };

  Port(Model* model);
  virtual ~Port();

  /// Interface to the /user/.
  /// Is implemented in PortAcceptor/PortProvider
  /// This function is responsible to distinguish into which end of the
  /// Connection object this Port belongs. This way the Connection object can
  /// make sure that it has exactly one PortProvider and exactly one
  /// PortAcceptor available to connect.
  virtual ConnectResult addConnection(Connection* connection) = 0;
  virtual bool removeConnection(Connection* connection) = 0;
  virtual void removeAllConnections() = 0;

  /// Sets the model it belongs to to zero and cuts all connections
  void invalidate();

  /// Return the model this port belongs to
  WeakPtr<Model> getModel() const;

private:
  WeakPtr<Model> mModel;
};

} // namespace OpenFDM

#endif
