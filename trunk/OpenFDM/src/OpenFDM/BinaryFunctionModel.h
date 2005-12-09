/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_BinaryFunctionModel_H
#define OpenFDM_BinaryFunctionModel_H

#include <string>

#include "Types.h"
#include "Property.h"
#include "Model.h"

namespace OpenFDM {

class BinaryFunctionModelImpl;

/// Class representing a model with exactly two inputs.
class BinaryFunctionModel :
    public Model {
public:
  enum Type {
    Atan2,
    Pow,
    Div
  };

  BinaryFunctionModel(const std::string& name, Type type);
  virtual ~BinaryFunctionModel(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const real_type& getFunctionValue(void) const;

  void setType(Type type);
  Type getType(void) const;

private:
  shared_ptr<BinaryFunctionModelImpl> mImpl;
  Type mType;
  real_type mFunctionValue;
};

} // namespace OpenFDM

#endif
