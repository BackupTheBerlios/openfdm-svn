/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_GroupInput_H
#define OpenFDM_GroupInput_H

#include "Model.h"
#include "ModelGroup.h"
#include "NumericPortProxy.h"

namespace OpenFDM {

class GroupInput : public Model {
  OPENFDM_OBJECT(GroupInput, Model);
public:
  GroupInput(const std::string& name);
  virtual ~GroupInput();
  
  virtual unsigned addParent(Group* model);
  virtual void removeParent(unsigned idx);

  NumericPortAcceptor* getGroupInput()
  { return mPortProxy; }

private:
  SharedPtr<NumericPortProxy> mPortProxy;
};

} // namespace OpenFDM

#endif
