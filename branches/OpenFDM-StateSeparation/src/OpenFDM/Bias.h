/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Bias_H
#define OpenFDM_Bias_H

#include <string>
#include <vector>

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Model.h"

namespace OpenFDM {

class Bias : public Model {
  OPENFDM_OBJECT(Bias, Model);
public:
  Bias(const std::string& name);
  virtual ~Bias(void);
  
  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const Matrix& getBias(void) const;
  void setBias(const Matrix& bias);

  const Matrix& getOutput(void) const;

private:
  Matrix mBias;
  Matrix mOutput;
  MatrixPortHandle mInputPort;
};

} // namespace OpenFDM

#endif
