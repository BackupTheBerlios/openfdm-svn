/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Integrator_H
#define OpenFDM_Integrator_H

#include "Types.h"
#include "Model.h"
#include "MatrixStateInfo.h"

namespace OpenFDM {

class Integrator : public Model {
  OPENFDM_OBJECT(Integrator, Model);
public:
  Integrator(const std::string& name);
  virtual ~Integrator(void);

  virtual bool alloc(LeafContext& leafContext) const;
  virtual void init(DiscreteStateValueVector& discreteState,
                    ContinousStateValueVector& continousState) const;

  virtual void output(const DiscreteStateValueVector&, const ContinousStateValueVector& continousState, PortValueList& portValues) const;
  virtual void derivative(const DiscreteStateValueVector&, const ContinousStateValueVector& state, const PortValueList& portValues, ContinousStateValueVector& deriv) const;

  virtual bool dependsOn(const PortId&, const PortId&) const;

  void setInitialValue(const Matrix& initialValue);
  const Matrix& getInitialValue() const;

private:
  MatrixInputPort mInputPort;
  MatrixOutputPort mOutputPort;
  MatrixInputPort mInitialValuePort;
  Matrix mInitialValue;

  // FIXME
  SharedPtr<MatrixStateInfo> mMatrixStateInfo;
};

} // namespace OpenFDM

#endif
