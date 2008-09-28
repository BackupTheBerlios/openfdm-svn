/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DiscreteIntegrator_H
#define OpenFDM_DiscreteIntegrator_H

#include "Types.h"
#include "Model.h"
#include "TemplateDiscreteStateInfo.h"

namespace OpenFDM {

class DiscreteIntegrator : public Model {
  OPENFDM_OBJECT(DiscreteIntegrator, Model);
public:
  DiscreteIntegrator(const std::string& name);
  virtual ~DiscreteIntegrator(void);

  virtual bool alloc(LeafContext& leafContext) const;
  virtual void init(DiscreteStateValueVector& discreteState,
                    ContinousStateValueVector&, const PortValueList&) const;
  virtual void output(const DiscreteStateValueVector& discreteState,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const;
  virtual void update(const DiscreteTask&, DiscreteStateValueVector&,
                      ContinousStateValueVector&, const PortValueList&) const;

  /// The initial output values on the output until input values are available.
  const Matrix& getInitialValue() const;
  void setInitialValue(const Matrix& initialValue);

  bool getEnableInitialValuePort() const;
  void setEnableInitialValuePort(bool enable);

  const Matrix& getMinSaturation(void) const;
  void setMinSaturation(const Matrix& value);

  const Matrix& getMaxSaturation(void) const;
  void setMaxSaturation(const Matrix& value);

private:
  void doUpdate(Matrix& integralValue, const Matrix& derivative,
                const real_type& dt) const;

  typedef TemplateDiscreteStateInfo<Matrix> MatrixStateInfo;

  MatrixInputPort mInputPort;
  MatrixOutputPort mOutputPort;
  MatrixInputPort mInitialValuePort;
  Matrix mInitialValue;
  Matrix mMinSaturation;
  Matrix mMaxSaturation;
  SharedPtr<MatrixStateInfo> mMatrixStateInfo;
};

} // namespace OpenFDM

#endif
