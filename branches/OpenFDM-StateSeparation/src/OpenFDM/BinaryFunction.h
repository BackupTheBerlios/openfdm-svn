/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_BinaryFunction_H
#define OpenFDM_BinaryFunction_H

#include <string>
#include "Model.h"

namespace OpenFDM {

/// Class representing a model with exactly two inputs.
class BinaryFunction : public Model {
  OPENFDM_OBJECT(BinaryFunction, Model);
public:
  enum Type {
    Atan2,
    Pow,
    Div
  };

  BinaryFunction(const std::string& name, Type type);
  virtual ~BinaryFunction(void);

  virtual bool alloc(ModelContext&) const;
  virtual void output(const Task&, const DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const;

  void setType(const Type& type);
  const Type& getType(void) const;

private:
  MatrixInputPort mInput0Port;
  MatrixInputPort mInput1Port;
  MatrixOutputPort mOutputPort;
  
  Type mType;
};

} // namespace OpenFDM

#endif
