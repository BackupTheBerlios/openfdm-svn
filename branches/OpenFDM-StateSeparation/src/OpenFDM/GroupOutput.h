/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_GroupOutput_H
#define OpenFDM_GroupOutput_H

#include <string>
#include "GroupInterfaceNode.h"
#include "Port.h"
#include "SharedPtr.h"

namespace OpenFDM {

class GroupOutput : public GroupInterfaceNode {
  OPENFDM_OBJECT(GroupOutput, GroupInterfaceNode);
public:
  GroupOutput(const std::string& name);
  virtual ~GroupOutput();

protected:
  virtual bool addParent(Node* parent);

private:
  SharedPtr<InputPort> mGroupInternalPort;
};

} // namespace OpenFDM

#endif
