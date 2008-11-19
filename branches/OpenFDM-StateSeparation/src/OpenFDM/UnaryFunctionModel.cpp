/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "UnaryFunctionModel.h"

#include <string>

#include "Types.h"

namespace OpenFDM {

class UnaryFunctionModelImpl :
    public Referenced {
public:
  virtual ~UnaryFunctionModelImpl(void) {}
  void setRealPortHandle(const RealPortHandle& realPortHandle)
  { mRealPortHandle = realPortHandle; }
  virtual real_type getValue(void) = 0;
protected:
  RealPortHandle mRealPortHandle;
};

/// Implementations of various similar functions.
class AbsUnaryFunctionModelImpl :
    public UnaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return fabs(mRealPortHandle.getRealValue()); }
};

class AcosUnaryFunctionModelImpl :
    public UnaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return acos(mRealPortHandle.getRealValue()); }
};

class AsinUnaryFunctionModelImpl :
    public UnaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return asin(mRealPortHandle.getRealValue()); }
};

class AtanUnaryFunctionModelImpl :
    public UnaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return atan(mRealPortHandle.getRealValue()); }
};

class CeilUnaryFunctionModelImpl :
    public UnaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return ceil(mRealPortHandle.getRealValue()); }
};

class CosUnaryFunctionModelImpl :
    public UnaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return cos(mRealPortHandle.getRealValue()); }
};

class ExpUnaryFunctionModelImpl :
    public UnaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return exp(mRealPortHandle.getRealValue()); }
};

class FloorUnaryFunctionModelImpl :
    public UnaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return floor(mRealPortHandle.getRealValue()); }
};

class LogUnaryFunctionModelImpl :
    public UnaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return log(mRealPortHandle.getRealValue()); }
};

class Log10UnaryFunctionModelImpl :
    public UnaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return log10(mRealPortHandle.getRealValue()); }
};

class MinusUnaryFunctionModelImpl :
    public UnaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return -mRealPortHandle.getRealValue(); }
};

class SinUnaryFunctionModelImpl :
    public UnaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return sin(mRealPortHandle.getRealValue()); }
};

class SqrUnaryFunctionModelImpl :
    public UnaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { real_type v = mRealPortHandle.getRealValue(); return v*v; }
};

class SqrtUnaryFunctionModelImpl :
    public UnaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return sqrt(mRealPortHandle.getRealValue()); }
};

class TanUnaryFunctionModelImpl :
    public UnaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return tan(mRealPortHandle.getRealValue()); }
};

BEGIN_OPENFDM_OBJECT_DEF(UnaryFunctionModel, Model)
  END_OPENFDM_OBJECT_DEF

UnaryFunctionModel::UnaryFunctionModel(const std::string& name, Type type) :
  Model(name),
  mType(type)
{
  setType(type);

  setDirectFeedThrough(true);

  setNumInputPorts(1);
  setInputPortName(0, "input");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &UnaryFunctionModel::getFunctionValue);
}

UnaryFunctionModel::~UnaryFunctionModel(void)
{
}

bool
UnaryFunctionModel::init(void)
{
  if (!mImpl) {
    Log(Model, Error) << "Initialization of UnaryFunctionModel model \""
                      << getName() << "\" failed: No funcion given!" << endl;
    return false;
  }

  RealPortHandle portHandle = getInputPort(0)->toRealPortHandle();
  if (!portHandle.isConnected()) {
    Log(Model, Error) << "Initialization of UnaryFunctionModel model \""
                      << getName()
                      << "\" failed: Input port \"" << getInputPortName(0)
                      << "\" is not connected!" << endl;
    return false;
  }
  mImpl->setRealPortHandle(portHandle);
  return Model::init();
}

void
UnaryFunctionModel::output(const TaskInfo&)
{
  // Evaluate the expression.
  mFunctionValue = mImpl->getValue();
}

const real_type&
UnaryFunctionModel::getFunctionValue(void) const
{
  return mFunctionValue;
}

void
UnaryFunctionModel::setType(UnaryFunctionModel::Type type)
{
  mType = type;
  switch (type) {
  case Abs:
    mImpl = new AbsUnaryFunctionModelImpl;
    break;
  case Acos:
    mImpl = new AcosUnaryFunctionModelImpl;
    break;
  case Asin:
    mImpl = new AsinUnaryFunctionModelImpl;
    break;
  case Atan:
    mImpl = new AtanUnaryFunctionModelImpl;
    break;
  case Ceil:
    mImpl = new CeilUnaryFunctionModelImpl;
    break;
  case Cos:
    mImpl = new CosUnaryFunctionModelImpl;
    break;
  case Exp:
    mImpl = new ExpUnaryFunctionModelImpl;
    break;
  case Floor:
    mImpl = new FloorUnaryFunctionModelImpl;
    break;
  case Log:
    mImpl = new LogUnaryFunctionModelImpl;
    break;
  case Log10:
    mImpl = new Log10UnaryFunctionModelImpl;
    break;
  case Minus:
    mImpl = new MinusUnaryFunctionModelImpl;
    break;
  case Sin:
    mImpl = new SinUnaryFunctionModelImpl;
    break;
  case Sqr:
    mImpl = new SqrUnaryFunctionModelImpl;
    break;
  case Sqrt:
    mImpl = new SqrtUnaryFunctionModelImpl;
    break;
  case Tan:
    mImpl = new TanUnaryFunctionModelImpl;
    break;
  default:
    break;
  }
}

UnaryFunctionModel::Type
UnaryFunctionModel::getType(void) const
{
  return mType;
}

} // namespace OpenFDM
