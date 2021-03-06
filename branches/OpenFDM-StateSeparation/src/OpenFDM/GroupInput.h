/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_GroupInput_H
#define OpenFDM_GroupInput_H

#include <string>
#include "GroupInterfaceNode.h"
#include "Port.h"
#include "SharedPtr.h"

namespace OpenFDM {

class GroupInput : public GroupInterfaceNode {
  OPENFDM_OBJECT(GroupInput, GroupInterfaceNode);
public:
  GroupInput(const std::string& name);
  virtual ~GroupInput();

protected:
  virtual bool addParent(Node* parent);

private:
  SharedPtr<OutputPort> mGroupInternalPort;
};

} // namespace OpenFDM

#endif
