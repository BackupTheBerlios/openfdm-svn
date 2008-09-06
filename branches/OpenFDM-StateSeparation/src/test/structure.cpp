#include <OpenFDM/Matrix.h>
#include <OpenFDM/Object.h>
#include <OpenFDM/SharedPtr.h>
#include <OpenFDM/WeakPtr.h>
#include <OpenFDM/Rotation.h>
#include <OpenFDM/Inertia.h>
#include <OpenFDM/PortValue.h>
#include <OpenFDM/PortValueList.h>
#include <OpenFDM/PortId.h>
#include <OpenFDM/PortInfo.h>
#include <OpenFDM/AcceptorPortInfo.h>
#include <OpenFDM/ProviderPortInfo.h>
#include <OpenFDM/NumericPortValue.h>
#include <OpenFDM/Node.h>
#include <OpenFDM/NodeVisitor.h>
#include <OpenFDM/NumericAcceptorPortInfo.h>
#include <OpenFDM/NumericProviderPortInfo.h>
#include <OpenFDM/MatrixInputPort.h>
#include <OpenFDM/MatrixOutputPort.h>
#include <OpenFDM/RealInputPort.h>
#include <OpenFDM/RealOutputPort.h>

#include <OpenFDM/StateInfo.h>
#include <OpenFDM/StateValue.h>
#include <OpenFDM/ContinousStateValue.h>
#include <OpenFDM/MatrixStateValue.h>

#include <OpenFDM/StateInfoVector.h>
#include <OpenFDM/ContinousStateInfoVector.h>

#include <OpenFDM/ContinousStateValueVector.h>
#include <OpenFDM/DiscreteStateValueVector.h>

#include <OpenFDM/BoolStateInfo.h>
#include <OpenFDM/RealStateInfo.h>
#include <OpenFDM/MatrixStateInfo.h>

#include <OpenFDM/LeafContext.h>

#include <OpenFDM/Gain.h>
#include <OpenFDM/Integrator.h>
#include <OpenFDM/Delay.h>

#include <OpenFDM/Group.h>

#include <iterator>
#include <algorithm>
#include <sstream>

namespace OpenFDM {

/// Evaluation orders:
///
/// Initialization:
///  init
///  output(t = t0)
/// Now the values in the ports belong to t = t0
///
/// Loop
///  while (t0 < tEnd)
///   output(t0) // Only if port values are not already at t0
///   update // prepare model's state for time interval [t0, t0+h]
///   Continous // do time integration
///    output(t0+s)
///    derivative(t0+s)
///
///  output(tEnd)
/// Now the values in the ports belong to t = tEnd
///  
/// Update discrete states based on ports at time t to discrete states for
/// the next timestep, that may just set state for t = t+h so that output can
/// now return values for the next timestep.
/// Then evaluate ports for time t = t+h.
///
/// Note that we can also outputs for the desired simulation time at the end.
/// Then we just reevaluate outputs when we get asked for a time within the
/// current integration timestep. If we need to start a new timestep,
/// reevaluate outputs for that particular timesteps start time.
/// In the best case this time matches the alread present output time.
/// FIXME: IMO THIS MUST WORK THIS WAY
///

class Print : public Model {
public:
  Print(const std::string& name = std::string()) :
    Model(name),
    mInputPort(newRealInputPort("input"))
  { }
  virtual void output(const DiscreteStateValueVector&,
                      const ContinousStateValueVector&,
                      PortValueList& portValues) const
  { std::cout << portValues[mInputPort] << std::endl; }
  virtual bool dependsOn(const PortId&, const PortId&) const
  { return false; }
private:
  RealInputPort mInputPort;
};

class LeafInstance : public WeakReferenced {
public:
  struct AcceptorPortData;
  struct ProviderPortData;

  class PortData : public WeakReferenced {
  public:
    PortData(LeafInstance* leafInstance, const PortInfo* portInfo) :
      mLeafInstance(leafInstance),
      mPortInfo(portInfo)
    { }
    virtual ~PortData() {}

    virtual AcceptorPortData* toAcceptorPortData()
    { return 0; }
    virtual ProviderPortData* toProviderPortData()
    { return 0; }
    
    /// Return the LeafInstance this PortData belongs to.
    SharedPtr<LeafInstance> getLeafInstance() const
    { return mLeafInstance.lock(); }

    const SharedPtr<const PortInfo>& getPortInfo() const
    { return mPortInfo; }

    virtual bool connect(PortData*) = 0;
    virtual bool merge(PortData*) = 0;

    virtual void print()
    { }

  private:
    WeakPtr<LeafInstance> mLeafInstance;
    SharedPtr<const PortInfo> mPortInfo;
  };

  struct ProviderPortData : public PortData {
    ProviderPortData(LeafInstance* leafInstance,
                     const ProviderPortInfo* providerPort) :
      PortData(leafInstance, providerPort),
      _providerPort(providerPort)
    { }
    virtual ProviderPortData* toProviderPortData()
    { return this; }

    virtual bool connect(PortData* portData)
    {
      if (!portData)
        return false;
      AcceptorPortData* acceptorPortData = portData->toAcceptorPortData();
      if (!acceptorPortData)
        return false;
      return acceptorPortData->connectToProvider(this);
    }

    virtual bool merge(PortData* portData)
    {
      if (!portData)
        return false;
      AcceptorPortData* acceptorPortData = portData->toAcceptorPortData();
      if (!acceptorPortData)
        return false;
      SharedPtr<const LeafInstance::ProviderPortData> providerPortData;
      providerPortData = acceptorPortData->_providerPortData.lock();
      OpenFDMAssert(providerPortData);

      // FIXME: think about that. Not yet completely functional

      _providerPort = providerPortData->_providerPort;
      // Merge must happen before the group this PortData belongs to is
      // connected
      OpenFDMAssert(_acceptorPortDataList.empty());
      _acceptorPortDataList = providerPortData->_acceptorPortDataList;
    }

    virtual void print()
    {
      std::cout << "  Provider Port \"" << getPortInfo()->getName()
                << "\" connected to:" << std::endl;
      for (unsigned i = 0; i < _acceptorPortDataList.size(); ++i) {
        SharedPtr<const AcceptorPortData> acceptorPortData;
        acceptorPortData = _acceptorPortDataList[i].lock();
        if (!acceptorPortData)
          continue;
        SharedPtr<LeafInstance> leafInstance;
        leafInstance = acceptorPortData->getLeafInstance();
        std::cout << "    Node \"" << leafInstance->getLeafNode()->getName()
                  << "\" Port \"" << acceptorPortData->getPortInfo()->getName()
                  << "\"" << std::endl;
      }
    }

    SharedPtr<const ProviderPortInfo> _providerPort;
    std::vector<WeakPtr<const AcceptorPortData> > _acceptorPortDataList;
  };
  struct AcceptorPortData : public PortData {
    AcceptorPortData(LeafInstance* leafInstance,
                     const AcceptorPortInfo* acceptorPort) :
      PortData(leafInstance, acceptorPort)
    { _acceptorPortList.push_back(acceptorPort); }
    virtual AcceptorPortData* toAcceptorPortData()
    { return this; }

    virtual bool connect(PortData* portData)
    {
      if (!portData)
        return false;
      ProviderPortData* providerPortData = portData->toProviderPortData();
      if (!providerPortData)
        return false;
      return connectToProvider(providerPortData);
    }

    bool connectToProvider(ProviderPortData* providerPortData)
    {
      // The current one must not be connected already ...
      OpenFDMAssert(!_providerPortData.lock());
      if (!providerPortData)
        return false;
      providerPortData->_acceptorPortDataList.push_back(this);
      _providerPortData = providerPortData;
      return true;
    }

    virtual bool merge(PortData* portData)
    {
      if (!portData)
        return false;
      ProviderPortData* providerPortData = portData->toProviderPortData();
      if (!providerPortData)
        return false;

      // FIXME: think about that. Not yet completely functional

      // Merge must happen before the group this PortData belongs to is
      // connected
      OpenFDMAssert(!_providerPortData.lock());

      unsigned i = 0;
      for (i = 0; i < providerPortData->_acceptorPortDataList.size(); ++i) {
        SharedPtr<const LeafInstance::AcceptorPortData> acceptorPortData;
        acceptorPortData = providerPortData->_acceptorPortDataList[i].lock();
        OpenFDMAssert(acceptorPortData);
        for (unsigned j = 0; j < _acceptorPortList.size(); ++j) {
          _acceptorPortList.push_back(acceptorPortData->_acceptorPortList[j]);
        }
      }
    }

    virtual void print()
    {
      // FIXME: seems to be a list???
      std::cout << "  Acceptor Port \"" << getPortInfo()->getName()
                << "\" connected from:" << std::endl;

      SharedPtr<const ProviderPortData> providerPortData;
      providerPortData = _providerPortData.lock();
      if (!providerPortData)
        return;
 
      SharedPtr<LeafInstance> leafInstance;
      leafInstance = providerPortData->getLeafInstance();
      std::cout << "    Node \"" << leafInstance->getLeafNode()->getName()
                << "\" Port \"" << providerPortData->getPortInfo()->getName()
                << "\"" << std::endl;
    }

    std::vector<SharedPtr<const AcceptorPortInfo> > _acceptorPortList;
    WeakPtr<const ProviderPortData> _providerPortData;
  };



  LeafInstance(const LeafNode* leaf) :
    mLeafNode(leaf)
  { allocPorts(leaf); }

  PortData* getPortData(const PortId& portId)
  {
    OpenFDMAssert(getLeafNode());
    unsigned index = getLeafNode()->getPortIndex(portId);
    // FIXME, is an error condition, not an assert???
    OpenFDMAssert(index < mPortData.size());
    return mPortData[index];
  }

  const SharedPtr<const LeafNode>& getLeafNode() const
  { return mLeafNode; }

  void print()
  {
    std::cout << "Leaf \"" << mLeafNode->getName() << "\"" << std::endl;
    for (unsigned i = 0; i < mPortData.size(); ++i) {
      mPortData[i]->print();
    }
  }

private:
  void allocPorts(const Node* node)
  {
    for (unsigned i = 0; i < node->getNumPorts(); ++i) {
      SharedPtr<const PortInfo> port = node->getPort(i);
      const ProviderPortInfo* providerPort = port->toProviderPortInfo();
      if (providerPort)
        mPortData.push_back(new ProviderPortData(this, providerPort));
      const AcceptorPortInfo* acceptorPort = port->toAcceptorPortInfo();
      if (acceptorPort)
        mPortData.push_back(new AcceptorPortData(this, acceptorPort));
    }
  }

  // The subsystem leaf node
  SharedPtr<const LeafNode> mLeafNode;

  // List of port dependent info used to build up the connect info and
  // the sorted list of leafs.
  std::vector<SharedPtr<PortData> > mPortData;
};

class LeafInstanceCollector : public NodeVisitor {
public:

  virtual void apply(Node& node)
  {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
  }

  // Aussen acceptor, innen provider
  virtual void apply(GroupAcceptorNode& leaf)
  {
    OpenFDMAssert(leaf.getPort(0));
    PortId portId = leaf.getPortId(0);
    LeafInstance::ProviderPortData* pd;
    pd = new LeafInstance::ProviderPortData(0, leaf._groupInternalPort);
    _leafPortDataMap[getCurrentNodeId()][portId] = pd;
  }
  // Aussen provider, innen acceptor
  virtual void apply(GroupProviderNode& leaf)
  {
    OpenFDMAssert(leaf.getPort(0));
    PortId portId = leaf.getPortId(0);
    LeafInstance::AcceptorPortData* ad;
    ad = new LeafInstance::AcceptorPortData(0, leaf._groupInternalPort);
    _leafPortDataMap[getCurrentNodeId()][portId] = ad;
  }
  virtual void apply(LeafNode& leaf)
  {
    // FIXME: assert that the current node id is something valid ...

    LeafInstance* leafInstance = new LeafInstance(&leaf);
    _leafInstanceList.push_back(leafInstance);

    for (unsigned i = 0; i < leaf.getNumPorts(); ++i) {
      PortId portId = leaf.getPortId(i);
      LeafInstance::PortData* portData = leafInstance->getPortData(portId);
      _leafPortDataMap[getCurrentNodeId()][portId] = portData;
    }
  }
  virtual void apply(Group& group)
  {
    // Prepare a new leaf map for the child group
    LeafPortDataMap parentLeafPortDataMap;
    parentLeafPortDataMap.swap(_leafPortDataMap);

    // Walk the children
#if 0
    group.traverse(*this);
#else
    for (unsigned i = 0; i < group.getNumChildren(); ++i) {
      pushNodeId(group.getNodeId(i));
      group.getChild(i)->accept(*this);
      popNodeId();
    }
#endif

    // Apply the group internal connections to the instances
    unsigned numConnects = group.getNumConnects();
    for (unsigned i = 0; i < numConnects; ++i) {
      Group::NodeId acceptorNodeId = group.getConnectAcceptorNodeId(i);
      Group::NodeId providerNodeId = group.getConnectProviderNodeId(i);

      if (!group.getChild(acceptorNodeId)) {
        std::cerr << "Cannot find acceptor node from nodeId" << std::endl;
        continue;
      }
      if (!group.getChild(acceptorNodeId)) {
        std::cerr << "Cannot find provider node from nodeId" << std::endl;
        continue;
      }

      SharedPtr<const AcceptorPortInfo> acceptorPort;
      acceptorPort = group.getConnectAcceptorPortInfo(i);
      PortId acceptorPortId = SharedPtr<const PortInfo>(acceptorPort);
      SharedPtr<const ProviderPortInfo> providerPort;
      providerPort = group.getConnectProviderPortInfo(i);
      PortId providerPortId = SharedPtr<const PortInfo>(providerPort);

      if (!acceptorPort) {
        std::cerr << "Cannot find acceptor Port data node "
                  << group.getChild(acceptorNodeId)->getName() << std::endl;
        continue;
      }
      if (!providerPort) {
        std::cerr << "Cannot find provider Port data node "
                  << group.getChild(providerNodeId)->getName() << std::endl;
        continue;
      }

      if (!_leafPortDataMap[acceptorNodeId][acceptorPortId]->
          connect(_leafPortDataMap[providerNodeId][providerPortId]))
        std::cerr << "Cannot connect????" << std::endl;
    }

    // Add a proxy LeafInstance that holds this groups ports, will be removed
    // when merged into the parent
    // FIXME: how to tell the parent group which connections need to be done??
    // FIXME:
    // add group connect routings
    // merge child list into the global list of instances
    for (unsigned i = 0; i < group.getNumPorts(); ++i) {
      PortId portId = group.getPortId(i);
      Group::NodeId nodeId = group.getGroupPortNode(portId);
      if (group.getPort(i)->toAcceptorPortInfo()) {
        LeafInstance::AcceptorPortData* ad;
        ad = new LeafInstance::AcceptorPortData(0, group.getPort(i)->toAcceptorPortInfo());
        parentLeafPortDataMap[getCurrentNodeId()][portId] = ad;
      }
      if (group.getPort(i)->toProviderPortInfo()) {
        LeafInstance::ProviderPortData* pd;
        pd = new LeafInstance::ProviderPortData(0, group.getPort(i)->toProviderPortInfo());
        parentLeafPortDataMap[getCurrentNodeId()][portId] = pd;
      }

      if (_leafPortDataMap[nodeId].empty()) {
        // FIXME, is this an internal error ???
        std::cerr << "Hmm, cannot find GroupPortNode for external port "
                  << i << std::endl;
        continue;
      }

      if (!parentLeafPortDataMap[getCurrentNodeId()][portId]->
          merge(_leafPortDataMap[nodeId].begin()->second))
        std::cerr << "Hmm, cannot merge port data" << std::endl;
    }

    parentLeafPortDataMap.swap(_leafPortDataMap);
  }

  ////////////////////////////////////////////////////////////////////////////
  // The final list of leafs we have in the system
  typedef std::list<SharedPtr<LeafInstance> > LeafInstanceList;
  LeafInstanceList _leafInstanceList;

  ////////////////////////////////////////////////////////////////////////////
  // Used to map connections in groups ...
  typedef std::map<PortId, SharedPtr<LeafInstance::PortData> > NodePortDataMap;
  typedef std::map<Group::NodeId, NodePortDataMap> LeafPortDataMap;
  LeafPortDataMap _leafPortDataMap;

  void pushNodeId(const Group::NodeId& nodeId)
  { _nodeIdStack.push_back(nodeId); }
  void popNodeId()
  { _nodeIdStack.pop_back(); }
  Group::NodeId getCurrentNodeId() const
  {
    if (_nodeIdStack.empty())
      return Group::NodeId();
    return _nodeIdStack.back();
  }

private:
  typedef std::list<Group::NodeId> NodeIdStack;
  NodeIdStack _nodeIdStack;
};

} // namespace OpenFDM

using namespace OpenFDM;

int main()
{
  SharedPtr<Group> group = new Group("G0");
  Group::NodeId gain = group->addChild(new Gain("gain"));
  Group::NodeId integrator1 = group->addChild(new Integrator("I1"));
  Group::NodeId integrator2 = group->addChild(new Integrator("I2"));
  Group::NodeId print = group->addChild(new Print("P"));
  Group::NodeId delay = group->addChild(new Delay("D"));
  Group::NodeId printDelay = group->addChild(new Print("PD"));

  group->connect(integrator1, "output", integrator2, "input");
  group->connect(integrator2, "output", gain, "input");
  group->connect(gain, "output", integrator1, "input");
  group->connect(integrator2, "output", print, "input");
  group->connect(gain, "output", delay, "input");
  group->connect(delay, "output", printDelay, "input");

  //FIXME: broken naming
//   Group::NodeId groupOutputNode = group->addAcceptorPort();
  Group::NodeId groupOutputNode = group->addProviderPort();
  group->connect(integrator2, "output", groupOutputNode, "input");

  SharedPtr<Group> topGroup = new Group("G1");
  Group::NodeId child0 = topGroup->addChild(group);
  Group::NodeId child1 = topGroup->addChild(group);

  Group::NodeId print0 = topGroup->addChild(new Print("P2"));
  topGroup->connect(child0, 0, print0, 0);
  Group::NodeId print1 = topGroup->addChild(new Print("P3"));
  topGroup->connect(child1, 0, print1, 0);

  /////////////////////////////////////////////////

  LeafInstanceCollector nodeInstanceCollector;
  topGroup->accept(nodeInstanceCollector);
  
  std::cout << nodeInstanceCollector._leafInstanceList.size() << std::endl;

  LeafInstanceCollector::LeafInstanceList::const_iterator i;
  for (i = nodeInstanceCollector._leafInstanceList.begin();
       i != nodeInstanceCollector._leafInstanceList.end();
       ++i) {
    (*i)->print();
  }

  return 0;
}
