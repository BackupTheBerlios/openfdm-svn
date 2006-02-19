/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_DeadBand_H
#define OpenFDM_DeadBand_H

#include <string>
#include <vector>

#include "Assert.h"
#include "Object.h"
#include "Model.h"

namespace OpenFDM {

class DeadBand : public Model {
  OPENFDM_OBJECT(DeadBand, Model);
public:
  DeadBand(const std::string& name);
  virtual ~DeadBand(void);
  
  virtual bool init(void);
  virtual void output(const TaskInfo&);

  const real_type& getWidth(void) const;
  void setWidth(const real_type& width);

  const real_type& getOutput(void) const;

private:
  real_type mWidth;
  real_type mOutput;
  RealPortHandle mInputPort;
};

} // namespace OpenFDM

#endif
