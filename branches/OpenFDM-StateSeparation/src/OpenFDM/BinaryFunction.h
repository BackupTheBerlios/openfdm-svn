/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_BinaryFunction_H
#define OpenFDM_BinaryFunction_H

#include <string>
#include "SimpleDirectModel.h"

namespace OpenFDM {

/// Class representing a model with exactly two inputs.
class BinaryFunction : public SimpleDirectModel {
  OPENFDM_OBJECT(BinaryFunction, SimpleDirectModel);
public:
  enum Type {
    Atan2,
    Pow,
    Div
  };

  BinaryFunction(const std::string& name, Type type);
  virtual ~BinaryFunction(void);

  virtual void output(Context& context) const;

  void setType(const Type& type);
  const Type& getType(void) const;

private:
  Type mType;
};

} // namespace OpenFDM

#endif
