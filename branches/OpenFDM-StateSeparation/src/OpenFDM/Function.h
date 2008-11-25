/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Function_H
#define OpenFDM_Function_H

#include "Object.h"
#include "Vector.h"
#include "Matrix.h"

namespace OpenFDM {

// FIXME: split that into two types of functions:
// Functions from R^m->R^n
// and ODE Functions ...

class Function
  : public Object {
public:
  typedef real_type              value_type;
  typedef LinAlg::size_type      size_type;
  typedef Vector                 invector_type;
  typedef Vector                 outvector_type;
  typedef Matrix                 jacobian_type;

  Function(void)
  { }
  virtual ~Function(void);

  virtual size_type inSize(void) const = 0;
  virtual size_type outSize(void) const = 0;
  virtual void eval(value_type t, const invector_type& v,
                    outvector_type& out) = 0;
  virtual void jac(value_type t, const invector_type& v, jacobian_type& jac);
  void numJac(value_type t, const invector_type& v, jacobian_type& jac);
};

} // namespace OpenFDM

#endif
