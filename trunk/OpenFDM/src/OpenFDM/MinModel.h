/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MinModel_H
#define OpenFDM_MinModel_H

#include <string>

#include "Matrix.h"
#include "Model.h"

namespace OpenFDM {

class MinModel : public Model {
  OPENFDM_OBJECT(MinModel, Model);
public:
  MinModel(const std::string& name);
  virtual ~MinModel(void);
  
  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const Matrix& getMin(void) const;

  unsigned getNumMinInputs(void) const;
  void setNumMinInputs(unsigned num);

private:
  Matrix mMin;
  std::vector<MatrixPortHandle> mInputPorts;
};

} // namespace OpenFDM

#endif
