/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include <string>

#include "Types.h"
#include "BinaryFunctionModel.h"

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
  OpenFDMAssert(mImpl);
  OpenFDMAssert(getInputPort(0)->isConnected());
  OpenFDMAssert(getInputPort(1)->isConnected());
  mImpl->setRealPortHandle(0, getInputPort(0)->toRealPortHandle());
  mImpl->setRealPortHandle(1, getInputPort(1)->toRealPortHandle());
  return getInputPort(0)->isConnected() && getInputPort(1)->isConnected();
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
