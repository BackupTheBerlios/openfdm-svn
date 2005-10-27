/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TransferFunction_H
#define OpenFDM_TransferFunction_H

#include "Types.h"
#include "Vector.h"
#include "Model.h"

namespace OpenFDM {

class DiscreteTransferFunction :
    public Model {
public:
  /// As always, we need a name in the constructor
  DiscreteTransferFunction(const std::string& name);
  virtual ~DiscreteTransferFunction(void);

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

  virtual bool init(void);
  virtual void output(const TaskInfo&);
  virtual void update(const TaskInfo& taskInfo);

  /// This one can have discrete states
  virtual void setDiscreteState(const Vector& state, unsigned offset);
  virtual void getDiscreteState(Vector& state, unsigned offset) const;

  const real_type& getOutput(void) const;

private:
  /// Holds the current output.
  real_type mOutput;
  /// Holds the denominator
  Vector mDen;
  /// Holds the numerator
  Vector mNum;

  /// Holds the normalized numerator and denominators
  Vector mDenNorm;
  Vector mNumNorm;
  real_type mD;
  /// Holds the interal discrete state
  Vector mState;

  /// FIXME, at the moment only explicit integration ...
};

} // namespace OpenFDM

#endif
