/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ExternalForceModel_H
#define OpenFDM_ExternalForceModel_H

#include "Assert.h"
#include "Object.h"
#include "Vector.h"
#include "Frame.h"
#include "Force.h"

namespace OpenFDM {

class ExternalForceModel
  : public ExternalForce {
public:
  ExternalForceModel(const std::string& name);
  virtual ~ExternalForceModel(void);

  virtual bool init(void);
  virtual void output(const TaskInfo&);

private:
  MatrixPortHandle mInputPort;
};

} // namespace OpenFDM

#endif
