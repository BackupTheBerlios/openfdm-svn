/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_GroupInterfaceNode_H
#define OpenFDM_GroupInterfaceNode_H

#include <string>
#include <vector>
#include <sstream>
#include "ConstNodeVisitor.h"
#include "Node.h"
#include "NodeVisitor.h"
#include "Object.h"
#include "Port.h"
#include "SharedPtr.h"

namespace OpenFDM {

class GroupInterfaceNode : public Node {
  OPENFDM_OBJECT(GroupInterfaceNode, Node);
public:
  GroupInterfaceNode(const std::string& name);
  virtual ~GroupInterfaceNode();

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  unsigned getExternalPortIndex() const;

  void setExternalPortName(const std::string& portName);
  const std::string& getExternalPortName() const;

protected:
  virtual bool addParent(Node* parent);
  virtual void removeParent(Node* parent);

  void setExternalPort(Port* portInfo);

private:
  SharedPtr<Port> mExternalPort;
};

} // namespace OpenFDM

#endif
