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

  virtual bool alloc(ModelContext&) const;
  virtual void init(const Task&, DiscreteStateValueVector& discreteState,
                    ContinousStateValueVector& continousState,
                    const PortValueList& portValueList) const;

  virtual void output(const Task&,const DiscreteStateValueVector&,
                      const ContinousStateValueVector& continousState,
                      PortValueList& portValues) const;
  virtual void derivative(const DiscreteStateValueVector&,
                          const ContinousStateValueVector& state,
                          const PortValueList& portValues,
                          ContinousStateValueVector& deriv) const;

  void setInitialValue(const Matrix& initialValue);
  void setInitialValue(const real_type& initialValue);
  const Matrix& getInitialValue() const;

  void setEnableInitialValuePort(bool enable);
  bool getEnableInitialValuePort() const;

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
