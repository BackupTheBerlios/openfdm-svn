/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_BinaryFunctionModel_H
#define OpenFDM_BinaryFunctionModel_H

#include <string>

#include "Types.h"
#include "Matrix.h"
#include "Property.h"
#include "Expression.h"
#include "Model.h"

namespace OpenFDM {

/// Class representing a model with exactly two inputs.
class BinaryFunctionModel :
    public Model {
public:
  BinaryFunctionModel(const std::string& name,
                      BinaryExpressionImpl<real_type>* expression);
  virtual ~BinaryFunctionModel(void);

  virtual bool init(void);
  virtual void output(void);

  const real_type& getFunctionValue(void) const;

private:
  virtual void inputPortChanged(unsigned i);

  shared_ptr<BinaryExpressionImpl<real_type> > mBinaryExpression;

  real_type mFunctionValue;
};

} // namespace OpenFDM

#endif
