/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ConstModel_H
#define OpenFDM_ConstModel_H

#include "Model.h"
#include "Matrix.h"

namespace OpenFDM {

class ConstModel : public Model {
  OPENFDM_OBJECT(ConstModel, Model);
public:
  ConstModel(const std::string& name, const Matrix& value = Matrix());
  ConstModel(const std::string& name, const real_type& value);
  virtual ~ConstModel(void);

  virtual void output(const Task&,const DiscreteStateValueVector&,
                      const ContinousStateValueVector& continousState,
                      PortValueList& portValues) const;

  const Matrix& getValue(void) const;
  void setValue(const Matrix& value);

  // For conveninence
  void setScalarValue(const real_type& value);

private:
  MatrixOutputPort mOutputPort;
  Matrix mValue;
};

} // namespace OpenFDM

#endif
