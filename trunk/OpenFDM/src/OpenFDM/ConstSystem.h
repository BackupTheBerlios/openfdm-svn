/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ConstSystem_H
#define OpenFDM_ConstSystem_H

#include "Model.h"
#include "Matrix.h"

namespace OpenFDM {

class ConstSystem
  : public Model {
public:
  ConstSystem(const std::string& name, const Matrix& value);
  virtual ~ConstSystem(void);

  const Matrix& getValue(void) const;
  void setValue(const Matrix& value);

private:
  Matrix mValue;
};

} // namespace OpenFDM

#endif
