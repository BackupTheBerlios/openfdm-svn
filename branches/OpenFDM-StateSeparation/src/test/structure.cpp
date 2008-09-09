#include <OpenFDM/Matrix.h>
#include <OpenFDM/Object.h>
#include <OpenFDM/SharedPtr.h>
#include <OpenFDM/WeakPtr.h>
#include <OpenFDM/Rotation.h>
#include <OpenFDM/Inertia.h>
#include <OpenFDM/PortValue.h>
#include <OpenFDM/PortValueList.h>
#include <OpenFDM/PortId.h>
#include <OpenFDM/Node.h>
#include <OpenFDM/NodeVisitor.h>
#include <OpenFDM/MatrixInputPort.h>
#include <OpenFDM/MatrixOutputPort.h>
#include <OpenFDM/RealInputPort.h>
#include <OpenFDM/RealOutputPort.h>
#include <OpenFDM/MechanicBodyPort.h>
#include <OpenFDM/MechanicInteractPort.h>

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

#include <OpenFDM/MechanicNode.h>

#include <OpenFDM/Gain.h>
#include <OpenFDM/Integrator.h>
#include <OpenFDM/Delay.h>

#include <OpenFDM/Group.h>

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
  Print(const std::string& name) :
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

class Body : public MechanicNode {
public:
  virtual void accept(NodeVisitor& visitor)
  { visitor.apply(*this); }

  // FIXME: is unclear: normal output port may depend on the velocities
  // or depend harder on the accelerations. The interface cannot ask for this
  // how about that??
//   enum OutputStage { Velocity, Articulation, Acceleration };
//   virtual bool dependsOn(OutputStage outputStage,
//                          const PortId& in, const PortId& out) const = 0;

protected:
  MechanicBodyPort
  newMechanicBodyPort(const std::string& name)
  { return MechanicBodyPort(this, name); }
};

class Interact : public MechanicNode {
public:
  virtual void accept(NodeVisitor& visitor)
  { visitor.apply(*this); }

  // FIXME: is unclear: normal output port may depend on the velocities
  // or depend harder on the accelerations. The interface cannot ask for this
  // how about that??
//   enum OutputStage { Velocity, Articulation, Acceleration };
//   virtual bool dependsOn(OutputStage outputStage,
//                          const PortId& in, const PortId& out) const = 0;

protected:
  MechanicInteractPort
  newMechanicInteractPort(const std::string& name)
  { return MechanicInteractPort(this, name); }
};








class LeafInstance : public WeakReferenced {
public:
  struct LeafPortData;
  struct AcceptorPortData;
  struct ProviderPortData;
  struct ProxyPortData;

  class PortData : public WeakReferenced {
  public:
    virtual ~PortData() {}
    virtual LeafPortData* toLeafPortData()
    { return 0; }
    virtual ProxyPortData* toProxyPortData()
    { return 0; }
    virtual AcceptorPortData* toAcceptorPortData()
    { return 0; }
    virtual ProviderPortData* toProviderPortData()
    { return 0; }
    virtual bool connect(PortData*) = 0;
  };

  class LeafPortData : public PortData {
  public:
    LeafPortData(LeafInstance* leafInstance, const PortInfo* portInfo) :
      mLeafInstance(leafInstance),
      mPortInfo(portInfo)
    { }
    virtual ~LeafPortData() {}

    virtual LeafPortData* toLeafPortData()
    { return this; }
    
    /// Return the LeafInstance this LeafPortData belongs to.
    SharedPtr<LeafInstance> getLeafInstance() const
    { return mLeafInstance.lock(); }

    const SharedPtr<const PortInfo>& getPortInfo() const
    { return mPortInfo; }

    virtual bool isConnected(LeafPortData*)
    { return false; }

    virtual void print()
    { }

  private:
    WeakPtr<LeafInstance> mLeafInstance;
    SharedPtr<const PortInfo> mPortInfo;
  };

  struct ProviderPortData : public LeafPortData {
    ProviderPortData(LeafInstance* leafInstance,
                     const ProviderPortInfo* providerPort) :
      LeafPortData(leafInstance, providerPort),
      _providerPort(providerPort)
    { }
    virtual ProviderPortData* toProviderPortData()
    { return this; }

    virtual bool connect(PortData* portData)
    {
      if (!portData)
        return false;
      ProxyPortData* proxyPortData = portData->toProxyPortData();
      if (proxyPortData)
        return proxyPortData->connect(this);
      AcceptorPortData* acceptorPortData = portData->toAcceptorPortData();
      if (!acceptorPortData)
        return false;
      return acceptorPortData->connectToProvider(this);
    }

    virtual bool isConnected(LeafPortData* portData)
    {
      if (!portData)
        return false;
      AcceptorPortData* acceptorPortData = portData->toAcceptorPortData();
      if (!acceptorPortData)
        return false;
      return acceptorPortData->isConnectedToProvider(this);
    }

    virtual void print()
    {
      std::cout << "  Provider Port \"" << getPortInfo()->getName()
                << "\" connected to:" << std::endl;
      for (unsigned i = 0; i < _acceptorPortDataList.size(); ++i) {
        SharedPtr<AcceptorPortData> acceptorPortData;
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
    std::vector<WeakPtr<AcceptorPortData> > _acceptorPortDataList;
  };
  struct AcceptorPortData : public LeafPortData {
    AcceptorPortData(LeafInstance* leafInstance,
                     const AcceptorPortInfo* acceptorPort) :
      LeafPortData(leafInstance, acceptorPort),
      _acceptorPort(acceptorPort)
    { }
    virtual AcceptorPortData* toAcceptorPortData()
    { return this; }

    virtual bool connect(PortData* portData)
    {
      if (!portData)
        return false;
      ProxyPortData* proxyPortData = portData->toProxyPortData();
      if (proxyPortData)
        return proxyPortData->connect(this);
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

    virtual bool isConnected(LeafPortData* portData)
    {
      if (!portData)
        return false;
      ProviderPortData* providerPortData = portData->toProviderPortData();
      if (!providerPortData)
        return false;
      return isConnectedToProvider(providerPortData);
    }

    bool isConnectedToProvider(ProviderPortData* providerPortData)
    {
      if (!providerPortData)
        return false;
      return providerPortData == _providerPortData.lock();
    }

    virtual void print()
    {
      std::cout << "  Acceptor Port \"" << getPortInfo()->getName()
                << "\" connected from:" << std::endl;

      SharedPtr<ProviderPortData> providerPortData;
      providerPortData = _providerPortData.lock();
      if (!providerPortData)
        return;
 
      SharedPtr<LeafInstance> leafInstance;
      leafInstance = providerPortData->getLeafInstance();
      std::cout << "    Node \"" << leafInstance->getLeafNode()->getName()
                << "\" Port \"" << providerPortData->getPortInfo()->getName()
                << "\"" << std::endl;
    }

    SharedPtr<const AcceptorPortInfo> _acceptorPort;
    WeakPtr<ProviderPortData> _providerPortData;
  };

  // Hmm, an other idea. How about treating the group input/output block
  // as a regular block with inputs and outputs?
  // Then in the output routine do nothing and just connect the PortValues of
  // the inputs directly to the PortValues of the outputs??
  // That would at least make the Proxy stuff simpler??

  class ProxyPortData : public PortData {
  public:
    ProxyPortData(PortData* portData)
    {
      if (!portData)
        return;
      AcceptorPortData* acceptorPortData = portData->toAcceptorPortData();
      if (acceptorPortData) {
        mPortDataList.push_back(acceptorPortData->_providerPortData.lock());
      }
      ProviderPortData* providerPortData = portData->toProviderPortData();
      if (providerPortData) {
        unsigned i;
        for (i = 0; i < providerPortData->_acceptorPortDataList.size(); ++i) {
          mPortDataList.push_back(providerPortData->_acceptorPortDataList[i].lock());
        }
      }
    }
    virtual ProxyPortData* toProxyPortData()
    { return this; }
    virtual bool connect(PortData* portData)
    {
      for (unsigned i = 0; i < mPortDataList.size(); ++i) {
        if (!mPortDataList[i]->connect(portData))
          return false;
      }
      return true;
    }
    std::vector<SharedPtr<PortData> > mPortDataList;
  };

  LeafInstance(const LeafNode* leaf) :
    mLeafNode(leaf)
  { allocPorts(leaf); }

  LeafPortData* getPortData(const PortId& portId)
  {
    OpenFDMAssert(getLeafNode());
    unsigned index = getLeafNode()->getPortIndex(portId);
    // FIXME, is an error condition, not an assert???
    OpenFDMAssert(index < mPortData.size());
    return mPortData[index];
  }

  const SharedPtr<const LeafNode>& getLeafNode() const
  { return mLeafNode; }

  // Return true if this leaf directly depends on one of leafInstance outputs
  bool dependsOn(LeafInstance* leafInstance)
  {
    for (unsigned i = 0; i < mPortData.size(); ++i) {
      for (unsigned j = 0; j < leafInstance->mPortData.size(); ++j) {
        if (!mPortData[i]->isConnected(leafInstance->mPortData[j]))
          continue;
        PortId inPortId = mLeafNode->getPortId(i);

        // FIXME, may be other concept:
        // make Model return a list of 'direct feedthrough ports'?
        for (unsigned k = 0; k < mPortData.size(); ++k) {
          PortId outPortId = mLeafNode->getPortId(k);
          if (mLeafNode->dependsOn(inPortId, outPortId))
            return true;
        }
      }
    }
    return false;
  }

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
  std::vector<SharedPtr<LeafPortData> > mPortData;
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
      LeafInstance::LeafPortData* portData = leafInstance->getPortData(portId);
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
      if (_leafPortDataMap[nodeId].empty()) {
        // FIXME, is this an internal error ???
        std::cerr << "Hmm, cannot find GroupPortNode for external port "
                  << i << std::endl;
        continue;
      }

      parentLeafPortDataMap[getCurrentNodeId()][portId] = 
          new LeafInstance::ProxyPortData(_leafPortDataMap[nodeId].begin()->second);
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


  // method to sort the leafs according to their dependency
  bool sortLeafList()
  {
    LeafInstanceList sortedLeafInstanceList;
    while (!_leafInstanceList.empty()) {
      SharedPtr<LeafInstance> leafInstance = _leafInstanceList.front();
      _leafInstanceList.pop_front();

      LeafInstanceList::iterator i;
      for (i = sortedLeafInstanceList.begin();
           i != sortedLeafInstanceList.end();
           ++i) {
        if (!(*i)->dependsOn(leafInstance))
          continue;
        sortedLeafInstanceList.insert(i, leafInstance);
        break;
      }
      if (i == sortedLeafInstanceList.end())
        sortedLeafInstanceList.push_back(leafInstance);
    }
    _leafInstanceList.swap(sortedLeafInstanceList);
  }

  
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
  
  nodeInstanceCollector.sortLeafList();

  std::cout << nodeInstanceCollector._leafInstanceList.size() << std::endl;

  LeafInstanceCollector::LeafInstanceList::const_iterator i;
  for (i = nodeInstanceCollector._leafInstanceList.begin();
       i != nodeInstanceCollector._leafInstanceList.end();
       ++i) {
    (*i)->print();
  }

  return 0;
}

// class CompositeNode : public Object /*??*/ {
//   // Might be something that behaves like a model, but depending on the input
//   // and output port types issues different final leafs scheduler ...

//   // FIXME: is it possible to make 'Library models' from that one?
//   // would be good, would simplify groups enormous
// };

// class LibraryNode;

// class LibraryModel : public Object {
// public:
//   LibraryModel(const std::string& name, Node* node = 0) :
//     Object(name),
//     mNode(node)
//   { }

//   unsigned getNumParents() const
//   { return mParents.size(); }
  
//   WeakPtr<LibraryNode> getParent(unsigned i)
//   { if (mParents.size() <= i) return 0; return mParents[i]; }
//   WeakPtr<const LibraryNode> getParent(unsigned i) const
//   { if (mParents.size() <= i) return 0; return mParents[i]; }

//   SharedPtr<Node> getNode()
//   { return mNode; }
//   SharedPtr<const Node> getNode() const
//   { return mNode; }

// private:
//   SharedPtr<Node> mNode;
//   std::vector<WeakPtr<LibraryNode> > mParents;
// };

// class LibraryNode : public Node {
// public:
//   LibraryNode(const std::string& name, LibraryModel* libraryModel = 0) :
//     Node(name),
//     mLibraryModel(libraryModel)
//   { }

//   // Hmm, how do we map ports??
//   // May be the Node just gets virtuals for ports???
//   // May be changing ports means informing the parent about that???

//   SharedPtr<LibraryModel> getLibraryModel()
//   { return mLibraryModel; }
//   SharedPtr<const LibraryModel> getLibraryModel() const
//   { return mLibraryModel; }

// private:
//   SharedPtr<LibraryModel> mLibraryModel;
// };
