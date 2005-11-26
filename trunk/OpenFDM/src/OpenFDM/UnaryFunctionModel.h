/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_UnaryFunctionModel_H
#define OpenFDM_UnaryFunctionModel_H

#include <string>

#include "Types.h"
#include "Matrix.h"
#include "Property.h"
#include "Expression.h"
#include "Model.h"

namespace OpenFDM {

/// Class representing a model with exactly one input.
class UnaryFunctionModel :
    public Model {
public:
  UnaryFunctionModel(const std::string& name,
                     UnaryExpressionImpl<real_type>* expression);
  virtual ~UnaryFunctionModel(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const real_type& getFunctionValue(void) const;

private:
  shared_ptr<UnaryExpressionImpl<real_type> > mUnaryExpression;

  real_type mFunctionValue;
};

} // namespace OpenFDM

#endif
