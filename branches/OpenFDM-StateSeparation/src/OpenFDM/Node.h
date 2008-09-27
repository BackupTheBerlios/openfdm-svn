/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Node_H
#define OpenFDM_Node_H

#include <string>
#include <vector>
#include "Object.h"
#include "PortId.h"
#include "PortInfo.h"
#include "SharedPtr.h"

namespace OpenFDM {

class Node;
class NodeVisitor;
class ConstNodeVisitor;

typedef std::vector<SharedPtr<const Node> > NodePath;

class Node : public Object {
  OPENFDM_OBJECT(Node, Object);
public:
  Node(const std::string& name);
  virtual ~Node();

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;
  void ascend(NodeVisitor& visitor);
  void ascend(ConstNodeVisitor& visitor) const;

  unsigned getNumParents() const
  { return mParentList.size(); }
  WeakPtr<const Node> getParent(unsigned i) const;
  WeakPtr<Node> getParent(unsigned i);

  SharedPtr<const PortInfo> getPort(const PortId& portId) const;
  SharedPtr<const PortInfo> getPort(unsigned index) const;
  SharedPtr<const PortInfo> getPort(const std::string& name) const;

  unsigned getNumPorts() const;
  PortId getPortId(unsigned index) const;
  PortId getPortId(const std::string& name) const;

  unsigned getPortIndex(const PortId& portId) const;
  bool checkPort(const PortId& portId) const;

protected:

  void addParent(Node* parent);
  void removeParent(Node* parent);

private:
  Node(const Node&);
  Node& operator=(const Node&);

  void addPort(PortInfo* port);
  void removePort(PortInfo* port);

  typedef std::vector<SharedPtr<PortInfo> > PortList;
  PortList mPortList;

  friend class PortInfo;

  typedef std::vector<WeakPtr<Node> > ParentList;
  ParentList mParentList;
};

} // namespace OpenFDM

#endif
