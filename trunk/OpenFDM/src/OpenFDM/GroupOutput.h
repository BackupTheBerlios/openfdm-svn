/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_GroupOutput_H
#define OpenFDM_GroupOutput_H

#include "Model.h"
#include "ModelGroup.h"
#include "NumericPortProxy.h"

namespace OpenFDM {

class GroupOutput : public Model {
  OPENFDM_OBJECT(GroupOutput, Model);
public:
  GroupOutput(const std::string& name);
  virtual ~GroupOutput();

  virtual unsigned addParent(Group* model);
  virtual void removeParent(unsigned idx);

  PortProvider* getGroupOutput()
  { return mPortProxy->getPortProvider(); }

private:
  SharedPtr<NumericPortProxy> mPortProxy;
};

} // namespace OpenFDM

#endif
