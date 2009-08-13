/* -*-c++-*- OpenFDM - Copyright (C) 2004-2009 Mathias Froehlich 
 *
 */

#include "JSBSimPropertyManager.h"

namespace OpenFDM {

class PortSet {
  struct PathPort {
    NodePath modelPath;
    SharedPtr<const Port> portProvider;
  };
public:
  PortSet(const Port* sourcePort = 0)
  {
    if (!sourcePort)
      return;
    PathPort pathPort;
    pathPort.portProvider = sourcePort;

    SharedPtr<const Node> node = sourcePort->getNode();
    if (node)
      // FIXME
      pathPort.modelPath = node->getNodePathList().front();

    mPortList.push_back(pathPort);
  }

  const Port* routeTo(const NodePath& path)
  {
    // could happen if the initialzer failed
    if (mPortList.empty())
      return 0;

    // ok, shortcut for old style connections
    if (path.empty())
      return mPortList.front().portProvider;

    const NodePath& originatingPath = mPortList.front().modelPath;
    // fast return if the models are not connected to the same root system
    if (path.front() != originatingPath.front())
      return 0;

    // first check, if we already have a route
    const Port* portProvider = findProvider(path);
    if (portProvider)
      return portProvider;

    // Compute the iterators for seperating the common part of the model path
    // from the different part
    NodePath::const_iterator mi1 = path.begin();
    NodePath::const_iterator mi2 = originatingPath.begin();
    while (mi1 != path.end() && mi2 != originatingPath.end()) {
      if (*mi1 != *mi2)
        break;
      ++mi1;
      ++mi2;
    }

    if (mi1 != path.end()) {
      // that is: we must first go up that path and search again
      NodePath pathUp = path;
      pathUp.pop_back();
      portProvider = routeTo(pathUp);
      if (!portProvider)
        return 0;
      
      GroupInput* groupInput = new GroupInput(portProvider->getName());
      Group* group = const_cast<Group*>(dynamic_cast<const Group*>(path.back().get()));
      group->addChild(groupInput);

      PathPort pathPort;
      pathPort.modelPath = groupInput->getNodePathList().front();
      pathPort.portProvider = group->getPort(groupInput->getExternalPortIndex());
      mPortList.push_back(pathPort);

      if (!group->connect(portProvider, groupInput->getPort("input")))
        return 0;
      
      return pathPort.portProvider.get();

    } else if (mi2 != originatingPath.end()) {
      // that is: we need to step deeper towards the origin of that port
      NodePath pathDown = path;
      pathDown.push_back(*mi2);
      portProvider = routeTo(pathDown);
      if (!portProvider)
        return 0;

      GroupOutput* groupOutput = new GroupOutput(portProvider->getName());
      Group* group = const_cast<Group*>(dynamic_cast<const Group*>(pathDown.back().get()));
      group->addChild(groupOutput);

      PathPort pathPort;
      pathPort.modelPath = groupOutput->getNodePathList().front();
      pathPort.portProvider = group->getPort(groupOutput->getExternalPortIndex());
      mPortList.push_back(pathPort);

      if (!group->connect(portProvider, groupOutput->getPort("input")))
        return 0;
      
      return pathPort.portProvider.get();

    } else {
      // should not happen, in this case the find provider must have been
      // successful,
      return 0;
    }
  }

  const Port* findProvider(const NodePath& path)
  {
    PortList::iterator i = mPortList.begin();
    while (i != mPortList.end()) {
      if (i->modelPath == path)
        return i->portProvider;
      ++i;
    }

    return 0;
  }

private:
  typedef std::list<PathPort> PortList;
  PortList mPortList;
};

/// Connect all the loose ends stored here
bool
JSBSimProperty::connect()
{
  if (!hasProviderPort())
    return false;
  
  PortSet portSet(mProvider.first);
  for (unsigned i = 0; i < mConsumers.size(); ++i) {
    SharedPtr<const Node> node = mConsumers[i].first->getNode();
    OpenFDMAssert(node);
    NodePathList nodePathList = node->getNodePathList();
    const Port* p = portSet.routeTo(nodePathList.front());
    if (!mConsumers[i].second->connect(p, mConsumers[i].first))
      return false;
  }
  
  return true;
}

} // namespace OpenFDM
