/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_UnaryFunction_H
#define OpenFDM_UnaryFunction_H

#include <string>
#include "SimpleDirectModel.h"

namespace OpenFDM {

/// Class representing a model with exactly one input.
class UnaryFunction : public SimpleDirectModel {
  OPENFDM_OBJECT(UnaryFunction, SimpleDirectModel);
public:
  enum Type {
    Abs,
    Acos,
    Asin,
    Atan,
    Ceil,
    Cos,
    Exp,
    Floor,
    Log,
    Log10,
    Minus,
    Sin,
    Sqr,
    Sqrt,
    Tan
  };

  UnaryFunction(const std::string& name, const Type& type);
  virtual ~UnaryFunction(void);

  void output(Context& context) const;

  void setType(const Type& type);
  const Type& getType(void) const;

private:
  Type mType;
};

} // namespace OpenFDM

#endif
