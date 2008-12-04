/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Group_H
#define OpenFDM_Group_H

#include <string>
#include <vector>
#include "Connect.h"
#include "Node.h"
#include "PortId.h"
#include "PortInfo.h"
#include "SharedPtr.h"

namespace OpenFDM {

class Group : public Node {
  OPENFDM_OBJECT(Group, Node);
public:
  Group(const std::string& name);
  virtual ~Group();

  /// Methods for the visitors.
  virtual void accept(NodeVisitor& visitor);
  virtual void accept(ConstNodeVisitor& visitor) const;

  /// Traverse this groups children with a visitor
  void traverse(NodeVisitor& visitor);
  void traverse(ConstNodeVisitor& visitor) const;

  /// Add a new child. Returns the number of this child wthin the group
  /// on success else ~0u is returned.
  unsigned addChild(Node* node);
  /// Remove the given child.
  void removeChild(const Node* node);
  void removeChild(unsigned i);
  /// Returns the number of children
  unsigned getNumChildren() const;
  /// Get child at index i.
  Node* getChild(unsigned i);
  /// Get child at index i.
  const Node* getChild(unsigned i) const;
  /// Get child number of the given node. If the node is not contained in
  /// the group ~0u is returned.
  unsigned getChildNumber(const Node* node) const;


  /// Connect api
  unsigned getNumConnects() const;
  Connect* getConnect(unsigned i);
  const Connect* getConnect(unsigned i) const;
  void removeConnect(unsigned i);
  void removeConnect(const Connect* connect);
  unsigned getConnectNumber(const Connect* connect) const;

  Connect* connect(const PortInfo* port0, const PortInfo* port1)
  {
    /// FIXME: more logs ...
    SharedPtr<Connect> connect = new Connect(this);
    if (!connect->setPortInfo0(port0))
      return 0;
    if (!connect->setPortInfo1(port1))
      return 0;
    mConnectList.push_back(connect);
    return connect.get();
  }


  unsigned getConnectNodeIndex0(unsigned i) const
  {
    if (getNumConnects() <= i)
      return ~0u;
    return getChildNumber(mConnectList[i]->getPortInfo0()->getNode());
  }
  unsigned getConnectNodeIndex1(unsigned i) const
  {
    if (getNumConnects() <= i)
      return ~0u;
    return getChildNumber(mConnectList[i]->getPortInfo1()->getNode());
  }

  SharedPtr<const PortInfo>
  getConnectPortInfo0(unsigned i) const
  {
    if (getNumConnects() <= i)
      return 0;
    return mConnectList[i]->getPortInfo0();
  }
  SharedPtr<const PortInfo>
  getConnectPortInfo1(unsigned i) const
  {
    if (getNumConnects() <= i)
      return 0;
    return mConnectList[i]->getPortInfo1();
  }

private:
  typedef std::vector<SharedPtr<Connect> > ConnectList;
  ConnectList mConnectList;

  typedef std::vector<SharedPtr<Node> > ChildList;
  ChildList mChildList;
};

} // namespace OpenFDM

#endif
