/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Node_H
#define OpenFDM_Node_H

#include <string>
#include <vector>
#include "Object.h"
#include "Port.h"
#include "SampleTime.h"
#include "SharedPtr.h"

namespace OpenFDM {

class Group;
class Node;
class NodeVisitor;
class ConstNodeVisitor;

typedef std::vector<SharedPtr<const Node> > NodePath;
typedef std::vector<NodePath> NodePathList;

class Node : public Object {
  OPENFDM_OBJECT(Node, Object);
public:
  Node(const std::string& name);
  virtual ~Node();

  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;
  void ascend(NodeVisitor& visitor);
  void ascend(ConstNodeVisitor& visitor) const;
  // Note the const in this method. only the nodes can change them
  void traversePorts(NodeVisitor& visitor) const;
  void traversePorts(ConstNodeVisitor& visitor) const;

  unsigned getNumParents() const
  { return mParentList.size(); }
  WeakPtr<const Node> getParent(unsigned i) const;
  WeakPtr<Node> getParent(unsigned i);
  bool isChildOf(const Node* node) const;

  unsigned getNumPorts() const;
  SharedPtr<const Port> getPort(unsigned index) const;
  SharedPtr<const Port> getPort(const std::string& name) const;
  unsigned getPortIndex(const Port* port) const;
  bool checkPort(const Port* port) const;

  const SampleTime& getSampleTime() const
  { return mSampleTime; }
  void setSampleTime(const SampleTime& sampleTime)
  { mSampleTime = sampleTime; }

  /// Return all node paths this Node is currently attached to.
  NodePathList getNodePathList() const;

  static std::string toNodePathName(const NodePath& nodePath)
  {
    NodePath::const_iterator i = nodePath.begin();
    if (i == nodePath.end())
      return std::string();

    std::string path = (*i)->getName();
    for (++i; i != nodePath.end(); ++i) {
      path += '/';
      path += (*i)->getName();
    }
    return path;
  }

protected:

  friend class Group;
  virtual bool addParent(Node* parent);
  virtual void removeParent(Node* parent);

private:
  Node(const Node&);
  Node& operator=(const Node&);

  /// Methods and variables for port handling.
  friend class Port;

  void addPort(Port* port);
  void removePort(Port* port);

  typedef std::vector<SharedPtr<Port> > PortList;
  PortList mPortList;

  /// Parents.
  typedef std::vector<WeakPtr<Node> > ParentList;
  ParentList mParentList;

  class CycleCheckVisitor;
  class NodePathListCollectVisitor;

  /// Sample time handling.
  SampleTime mSampleTime;
};

} // namespace OpenFDM

#endif
