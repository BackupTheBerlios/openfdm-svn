/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MaxModel_H
#define OpenFDM_MaxModel_H

#include <string>

#include "Matrix.h"
#include "Model.h"

namespace OpenFDM {

class MaxModel :
    public Model {
public:
  MaxModel(const std::string& name);
  virtual ~MaxModel(void);
  
  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const Matrix& getMax(void) const;

  unsigned getNumMaxInputs(void) const;
  void setNumMaxInputs(const unsigned& num);

private:
  Matrix mMax;
};

} // namespace OpenFDM

#endif
