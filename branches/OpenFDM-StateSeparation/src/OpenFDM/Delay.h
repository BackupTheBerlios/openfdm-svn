/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Delay_H
#define OpenFDM_Delay_H

#include <string>

#include "Model.h"
#include "TemplateDiscreteStateInfo.h"

namespace OpenFDM {

class Delay : public Model {
  OPENFDM_OBJECT(Delay, Model);
public:
  Delay(const std::string& name);
  virtual ~Delay();

  virtual bool alloc(LeafContext& leafContext) const;
  virtual void init(DiscreteStateValueVector& discreteState,
                    ContinousStateValueVector&, const PortValueList&) const;
  virtual void output(const DiscreteStateValueVector& discreteState,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const;
  virtual void update(DiscreteStateValueVector& discreteState,
                      ContinousStateValueVector&,
                      const PortValueList& portValues) const;

  /// The delay number of timeslices
  unsigned getDelay(void) const;
  void setDelay(unsigned delay);

  /// The initial output values on the output until input values are available.
  const Matrix& getInitialValue() const;
  void setInitialValue(const Matrix& initialValue);

private:

  class MatrixList : public std::list<Matrix> {
  public:
    MatrixList() {}
    MatrixList(size_type n, const Matrix& matrix)
    {
      while (0 < n--)
        push_back(matrix);
    }
    void rotate(const Matrix& matrix)
    {
      // splice is O(1)
      splice(end(), *this, begin());
      // append new value
      back() = matrix;
    }
  };

  typedef  TemplateDiscreteStateInfo<MatrixList> MatrixListStateInfo;

  MatrixInputPort mInputPort;
  MatrixOutputPort mOutputPort;
  unsigned mDelay;
  Matrix mInitialValue;
  SharedPtr<MatrixListStateInfo> mMatrixStateInfo;
};

} // namespace OpenFDM

#endif
