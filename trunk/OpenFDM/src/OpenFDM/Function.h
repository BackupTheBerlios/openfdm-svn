/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Function_H
#define OpenFDM_Function_H

#include "Object.h"
#include "Vector.h"
#include "Matrix.h"

namespace OpenFDM {

// FIXME: aplit that into two types of functions:
// Functions from R^m->R^n
// and ODE Functions ...

class Function
  : public Object {
public:
  enum Linearity {
    Linear,
    Piecewise,
    Nonlinear
  };

  typedef real_type              value_type;
  typedef LinAlg::size_type      size_type;
  typedef Vector                 invector_type;
  typedef Vector                 outvector_type;
  typedef Matrix                 jacobian_type;

  Function(Linearity linearity = Nonlinear)
    : linearity_(linearity)
  { }
  virtual ~Function(void);

  virtual size_type inSize(void) const = 0;
  virtual size_type outSize(void) const = 0;
  virtual void eval(value_type t, const invector_type& v,
                    outvector_type& out) = 0;
  virtual void jac(value_type t, const invector_type& v, jacobian_type& jac);
  void numJac(value_type t, const invector_type& v, jacobian_type& jac);

  virtual void output(void);
  virtual void update(value_type dt);

  Linearity linearity(void) const
  { return linearity_; }
  bool linear(void) const
  { return linearity_ == Linear; }
  bool piecewiseLinear(void) const
  { return linearity_ != Nonlinear; }
  bool nonLinear(void) const
  { return linearity_ == Nonlinear; }

protected:
  void setLinearity(Linearity linearity)
  { linearity_ = linearity; }

private:
  Linearity linearity_;
};

} // namespace OpenFDM

#endif
