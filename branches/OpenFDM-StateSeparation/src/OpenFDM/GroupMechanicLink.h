/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_GroupMechanicLink_H
#define OpenFDM_GroupMechanicLink_H

#include <string>
#include "GroupInterfaceNode.h"
#include "Port.h"
#include "SharedPtr.h"

namespace OpenFDM {

class GroupMechanicLink : public GroupInterfaceNode {
  OPENFDM_OBJECT(GroupMechanicLink, GroupInterfaceNode);
public:
  GroupMechanicLink(const std::string& name);
  virtual ~GroupMechanicLink();

protected:
  virtual bool addParent(Node* parent);

private:
  SharedPtr<MechanicLink> mGroupInternalPort;
};

} // namespace OpenFDM

#endif
