/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Gain_H
#define OpenFDM_Gain_H

#include <string>
#include <vector>

#include "Assert.h"
#include "Object.h"
#include "Property.h"
#include "Vector.h"
#include "Model.h"

namespace OpenFDM {

class Gain : public Model {
public:
  Gain(const std::string& name);
  virtual ~Gain(void);
  
  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const real_type& getGain(void) const;
  void setGain(const real_type& gain);

  const Matrix& getOutput(void) const;

private:
  real_type mGain;
  Matrix mOutput;
};

} // namespace OpenFDM

#endif
