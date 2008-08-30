/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "Integrator.h"

#include "Assert.h"
#include "LeafContext.h"

namespace OpenFDM {

BEGIN_OPENFDM_OBJECT_DEF(Integrator, Model)
  DEF_OPENFDM_PROPERTY(Matrix, InitialValue, Serialized)
  END_OPENFDM_OBJECT_DEF

Integrator::Integrator(const std::string& name = std::string()) :
  Model(name),
  mInputPort(newMatrixInputPort("input")),
  mOutputPort(newMatrixOutputPort("output")),
  mInitialValuePort(newMatrixInputPort("initialValue"))
{
  mMatrixStateInfo = new MatrixStateInfo;
  addContinousStateInfo(mMatrixStateInfo);
}

Integrator::~Integrator(void)
{
}

bool
Integrator::alloc(LeafContext& leafContext) const
{
  Size sz = size(leafContext.mPortValueList[mInputPort]);
  leafContext.mPortValueList.setPortSize(mOutputPort, sz);
  leafContext.mContinousState.setValue(*mMatrixStateInfo, leafContext);
  return true;
}

void
Integrator::init(DiscreteStateValueVector& discreteState,
                 ContinousStateValueVector& continousState) const
{
  // Needs to be done here. Need port values???
  // FIXME, can I ensure that at least the direct dependent ones are
  // available and the other ones are inaccessible at compile time?
//     if (portValues.isConnected(mInitialValuePort)) {
//       // external initial condition
//       continousState[*mMatrixStateInfo] = portValues[mInitialValuePort];
//     } else {
    // internal initial condition
    continousState[*mMatrixStateInfo].clear(); // FIXME
//     }
}

void
Integrator::output(const DiscreteStateValueVector&,
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

bool
Integrator::dependsOn(const PortId&, const PortId&) const
{
  // FIXME, make the initial value port depend here ...
  return false;
}

void
Integrator::setInitialValue(const Matrix& initialValue)
{
  mInitialValue = initialValue;
}

const Matrix&
Integrator::getInitialValue() const
{
  return mInitialValue;
}

} // namespace OpenFDM
