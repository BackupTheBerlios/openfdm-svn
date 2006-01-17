/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
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
  virtual ~ConstModel(void);

  const Matrix& getValue(void) const;
  void setValue(const Matrix& value);

private:
  Matrix mValue;
};

} // namespace OpenFDM

#endif
