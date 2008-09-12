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

class NodeVisitor;
class ConstNodeVisitor;

class Node : public Object {
  OPENFDM_OBJECT(Node, Object);
public:
  Node(const std::string& name = std::string());
  virtual ~Node();

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  SharedPtr<const PortInfo> getPort(const PortId& portId) const;
  SharedPtr<const PortInfo> getPort(unsigned index) const;
  SharedPtr<const PortInfo> getPort(const std::string& name) const;

  unsigned getNumPorts() const;
  PortId getPortId(unsigned index) const;
  PortId getPortId(const std::string& name) const;

  unsigned getPortIndex(const PortId& portId) const;
  bool checkPort(const PortId& portId) const;

protected:

private:
  Node(const Node&);
  Node& operator=(const Node&);

  void addPort(PortInfo* port);
  void removePort(PortInfo* port);

  typedef std::vector<SharedPtr<PortInfo> > PortList;
  PortList mPortList;

  friend class PortInfo;
};

} // namespace OpenFDM

#endif
