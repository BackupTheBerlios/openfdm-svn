/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "Integrator.h"

#include "Assert.h"
#include "ModelContext.h"
#include "TypeInfo.h"
#include "Variant.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Integrator, Model)
  DEF_OPENFDM_PROPERTY(Matrix, InitialValue, Serialized)
  DEF_OPENFDM_PROPERTY(Bool, EnableInitialValuePort, Serialized)
  END_OPENFDM_OBJECT_DEF

Integrator::Integrator(const std::string& name = std::string()) :
  Model(name),
  mInputPort(newMatrixInputPort("input", false)),
  mOutputPort(newMatrixOutputPort("output")),
  mInitialValue(Matrix::zeros(1, 1))
{
  mMatrixStateInfo = new MatrixStateInfo;
  addContinousStateInfo(mMatrixStateInfo);
}

Integrator::~Integrator(void)
{
}

bool
Integrator::alloc(ModelContext& context) const
{
  Size sz;
  if (getEnableInitialValuePort()) {
    sz = size(context.getPortValueList()[mInitialValuePort]);
    Log(Initialization, Debug)
      << "Size for Integrator is detemined by the initial input "
      << "port with size: " << trans(sz) << std::endl;
  } else {
    sz = size(mInitialValue);
    Log(Initialization, Debug)
      << "Size for Integrator is detemined by the static initial value "
      << "with size: " << trans(sz) << std::endl;
  }
  if (!context.getPortValueList().setOrCheckPortSize(mInputPort, sz)) {
    Log(Initialization, Error)
      << "Size for input port does not match!" << std::endl;
    return false;
  }
  if (!context.getPortValueList().setOrCheckPortSize(mOutputPort, sz)) {
    Log(Initialization, Error)
      << "Size for input port does not match!" << std::endl;
    return false;
  }
  context.setContinousStateSize(*mMatrixStateInfo, sz);
  return true;
}

void
Integrator::init(const Task&, DiscreteStateValueVector&,
                 ContinousStateValueVector& continousState,
                 const PortValueList& portValues) const
{
  if (getEnableInitialValuePort()) {
    // external initial condition
    continousState[*mMatrixStateInfo] = portValues[mInitialValuePort];
  } else {
    // internal initial condition
    continousState[*mMatrixStateInfo] = mInitialValue;
  }
}

void
Integrator::output(const Task&,const DiscreteStateValueVector&,
                   const ContinousStateValueVector& continousState,
                   PortValueList& portValues) const
{
  portValues[mOutputPort] = continousState[*mMatrixStateInfo];
}

void
Integrator::derivative(const DiscreteStateValueVector&,
                       const ContinousStateValueVector& state,
                       const PortValueList& portValues,
                       ContinousStateValueVector& deriv) const
{
  deriv[*mMatrixStateInfo] = portValues[mInputPort];
}

void
Integrator::setInitialValue(const Matrix& initialValue)
{
  mInitialValue = initialValue;
}

void
Integrator::setInitialValue(const real_type& initialValue)
{
  mInitialValue.resize(1, 1);
  mInitialValue(0, 0) = initialValue;
}

const Matrix&
Integrator::getInitialValue() const
{
  return mInitialValue;
}

void
Integrator::setEnableInitialValuePort(bool enable)
{
  if (enable == getEnableInitialValuePort())
    return;

  if (enable)
    mInitialValuePort = newMatrixInputPort("initialValue", true);
  else 
    mInitialValuePort.clear();
}

bool
Integrator::getEnableInitialValuePort() const
{
  return !mInitialValuePort.empty();
}

} // namespace OpenFDM
