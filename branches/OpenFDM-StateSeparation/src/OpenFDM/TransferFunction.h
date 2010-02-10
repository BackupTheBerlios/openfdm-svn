/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TransferFunction_H
#define OpenFDM_TransferFunction_H

#include "Types.h"
#include "Vector.h"
#include "Model.h"
#include "TemplateDiscreteStateInfo.h"

namespace OpenFDM {

class DiscreteTransferFunction : public Model {
  OPENFDM_OBJECT(DiscreteTransferFunction, Model);
public:
  /// As always, we need a name in the constructor
  DiscreteTransferFunction(const std::string& name);
  virtual ~DiscreteTransferFunction(void);

  virtual bool alloc(ModelContext&) const;
  virtual void init(const Task&,DiscreteStateValueVector& discreteState,
                    ContinousStateValueVector&, const PortValueList&) const;
  virtual void output(const Task&,const DiscreteStateValueVector& discreteState,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const;
  virtual void update(const DiscreteTask&, DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      const PortValueList&) const;

  /// Sets the coefficients for the denominator polynomial starting with the
  /// highest power's coefficient in the first vector element
  void setDenominator(const Vector& den);
  /// Returns the coefficients for the denominator polynomial starting with the
  /// highest power's coefficient in the first vector element
  const Vector& getDenominator(void) const;

  /// Sets the coefficients for the numerator polynomial starting with the
  /// highest power's coefficient in the first vector element
  /// Note that the coefficients are not assumed to be normalized, that is,
  /// The highest coefficient is not assumed to be implicitly 1
  void setNumerator(const Vector& num);
  /// Returns the coefficients for the numerator polynomial starting with the
  /// highest power's coefficient in the first vector element
  const Vector& getNumerator(void) const;

private:
//   MatrixInputPort mInputPort;
//   MatrixOutputPort mOutputPort;
  RealOutputPort mOutputPort;
  RealInputPort mInputPort;
  typedef TemplateDiscreteStateInfo<Matrix> MatrixStateInfo;
  SharedPtr<MatrixStateInfo> mMatrixStateInfo;

  /// Holds the denominator
  Vector mDen;
  /// Holds the numerator
  Vector mNum;

  /// Holds the normalized numerator and denominators
  /// FIXME, avoid the mutable ...
  mutable Vector mDenNorm;
  mutable Vector mNumNorm;
  mutable real_type mD;
};

} // namespace OpenFDM

#endif
