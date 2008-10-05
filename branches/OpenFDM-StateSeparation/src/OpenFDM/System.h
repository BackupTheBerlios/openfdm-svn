/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_System_H
#define OpenFDM_System_H

#include <string>

#include "AbstractNodeInstance.h"
#include "Node.h"
#include "Object.h"

namespace OpenFDM {

/// The System is the top \ref Node for a simulation system.
/// Provides algorithms to simulate and trim the whole system.

class AbstractSystem;

class System : public Object {
  OPENFDM_OBJECT(System, Object);
public:
  System(const std::string& name, Node* node = 0);
  virtual ~System();

  SharedPtr<Node> getNode() { return mNode; }
  SharedPtr<const Node> getNode() const { return mNode; }
  void setNode(Node* node);

  bool init();
  void clear();

  /// Simulate the system until the time tEnd
  bool simulate(const real_type& t);

  /// Bring the system in an equilibrum state near the current state ...
  bool trim(void);

  /// Return the current simulation time, convenience function
  real_type getTime(void) const;

  const ConstNodeInstanceList& getNodeInstanceList() const
  { return mNodeInstanceList; }

  const AbstractNodeInstance* getNodeInstance(const NodePath& nodePath) const
  {
    NodeInstanceMap::const_iterator i = mNodeInstanceMap.find(nodePath);
    if (i == mNodeInstanceMap.end())
      return 0;
    return i->second;
  }
  AbstractNodeInstance* getNodeInstance(const NodePath& nodePath)
  {
    NodeInstanceMap::const_iterator i = mNodeInstanceMap.find(nodePath);
    if (i == mNodeInstanceMap.end())
      return 0;
    return i->second;
  }

private:
  class NodeInstanceCollector;

  SharedPtr<Node> mNode;

  SharedPtr<AbstractSystem> mAbstractSystem;
  ConstNodeInstanceList mNodeInstanceList;
  NodeInstanceMap mNodeInstanceMap;
};

} // namespace OpenFDM

#endif
