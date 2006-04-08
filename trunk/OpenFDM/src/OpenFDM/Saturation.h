/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Saturation_H
#define OpenFDM_Saturation_H

#include <string>
#include <vector>

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Matrix.h"
#include "Model.h"

namespace OpenFDM {

class Saturation : public Model {
  OPENFDM_OBJECT(Saturation, Model);
public:
  Saturation(const std::string& name);
  virtual ~Saturation(void);
  
  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const Matrix& getMinSaturation(void) const;
  void setMinSaturation(const Matrix& minSaturation);
  const Matrix& getMaxSaturation(void) const;
  void setMaxSaturation(const Matrix& maxSaturation);

  const Matrix& getOutput(void) const;

private:
  Matrix mMaxSaturation;
  Matrix mMinSaturation;
  Matrix mOutput;

  MatrixPortHandle mInputPort;
};

} // namespace OpenFDM

#endif
