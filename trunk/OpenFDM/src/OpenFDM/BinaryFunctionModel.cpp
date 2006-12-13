/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "BinaryFunctionModel.h"

#include <string>
#include "Types.h"

namespace OpenFDM {

class BinaryFunctionModelImpl :
    public Referenced {
public:
  virtual ~BinaryFunctionModelImpl(void) {}
  void setRealPortHandle(unsigned idx, const RealPortHandle& realPortHandle)
  { mRealPortHandle[idx] = realPortHandle; }
  virtual real_type getValue(void) = 0;
protected:
  RealPortHandle mRealPortHandle[2];
};

class Atan2BinaryFunctionModelImpl :
    public BinaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return atan2(mRealPortHandle[0].getRealValue(), mRealPortHandle[1].getRealValue()); }
};

class PowBinaryFunctionModelImpl :
    public BinaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return pow(mRealPortHandle[0].getRealValue(), mRealPortHandle[1].getRealValue()); }
};

class DivBinaryFunctionModelImpl :
    public BinaryFunctionModelImpl {
private:
  virtual real_type getValue(void)
  { return mRealPortHandle[0].getRealValue() / mRealPortHandle[1].getRealValue(); }
};


BEGIN_OPENFDM_OBJECT_DEF(BinaryFunctionModel, Model)
  END_OPENFDM_OBJECT_DEF

BinaryFunctionModel::BinaryFunctionModel(const std::string& name, Type type) :
  Model(name)
{
  setType(type);

  setDirectFeedThrough(true);

  setNumInputPorts(2);
  setInputPortName(0, "input 0");
  setInputPortName(1, "input 1");
  
  setNumOutputPorts(1);
  setOutputPort(0, "output", this, &BinaryFunctionModel::getFunctionValue);
}

BinaryFunctionModel::~BinaryFunctionModel(void)
{
}

bool
BinaryFunctionModel::init(void)
{
  if (!mImpl) {
    Log(Model, Error) << "Initialization of BinaryFunctionModel model \""
                      << getName() << "\" failed: No funcion given!" << endl;
    return false;
  }

  RealPortHandle portHandle = getInputPort(0)->toRealPortHandle();
  if (!portHandle.isConnected()) {
    Log(Model, Error) << "Initialization of BinaryFunctionModel model \""
                      << getName()
                      << "\" failed: Input port \"" << getInputPortName(0)
                      << "\" is not connected!" << endl;
    return false;
  }
  mImpl->setRealPortHandle(0, portHandle);

  portHandle = getInputPort(1)->toRealPortHandle();
  if (!portHandle.isConnected()) {
    Log(Model, Error) << "Initialization of BinaryFunctionModel model \""
                      << getName()
                      << "\" failed: Input port \"" << getInputPortName(1)
                      << "\" is not connected!" << endl;
    return false;
  }
  mImpl->setRealPortHandle(1, portHandle);

  return Model::init();
}

void
BinaryFunctionModel::output(const TaskInfo&)
{
  // Evaluate the expression.
  mFunctionValue = mImpl->getValue();
}

const real_type&
BinaryFunctionModel::getFunctionValue(void) const
{
  return mFunctionValue;
}

void
BinaryFunctionModel::setType(BinaryFunctionModel::Type type)
{
  mType = type;
  switch (type) {
  case Atan2:
    mImpl = new Atan2BinaryFunctionModelImpl;
    break;
  case Pow:
    mImpl = new PowBinaryFunctionModelImpl;
    break;
  case Div:
    mImpl = new DivBinaryFunctionModelImpl;
    break;
  default:
    break;
  }
}

BinaryFunctionModel::Type
BinaryFunctionModel::getType(void) const
{
  return mType;
}

} // namespace OpenFDM
