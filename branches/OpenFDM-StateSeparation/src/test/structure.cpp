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
#include <OpenFDM/RootJoint.h>

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

#include <OpenFDM/RigidBody.h>
#include <OpenFDM/Interact.h>

#include <OpenFDM/Gain.h>
#include <OpenFDM/Integrator.h>
#include <OpenFDM/Delay.h>
#include <OpenFDM/Output.h>

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



///
/// ----------------
/// | LeafInstance |
/// ----------------
///             | |     ------------
///             | ----->| PortData |
///             |       ------------
///             |       --------------------------------
///             ------->| ModelContext/MechanicContext |
///                     --------------------------------
///
/// The LeafInstanceCollector walks the graph of models and collects
/// the leaf nodes in the system. For each leaf node there is a
/// LeafInstance class. Each LeafInstance has a list of PortData pointers
/// that is used to connect the Ports of leafs. Also there is a LeafContext
/// in each instance that is later needed to run the model.
///

class ModelContext : public Referenced {
public:
  ModelContext(const Model* model) :
    mModel(model)
  { }

  bool alloc()
  { return mModel->alloc(mLeafContext); }
  void init()
  { mModel->init(mLeafContext.mDiscreteState, mLeafContext.mContinousState); }
  void output()
  { mModel->output(mLeafContext.mDiscreteState, mLeafContext.mContinousState,
                   mLeafContext.mPortValueList); }
  void update()
  { mModel->update(mLeafContext.mDiscreteState, mLeafContext.mContinousState,
                   mLeafContext.mPortValueList); }

//   void derivative()
//   { mModel->derivative(mLeafContext.mDiscreteState,
//                        mLeafContext.mContinousState,
//                        mLeafContext.mPortValueList,
//                        mLeafContext.mContinousStateDerivative); }

  SharedPtr<const Model> mModel;
  LeafContext mLeafContext;

private:
  ModelContext();
  ModelContext(const ModelContext&);
  ModelContext& operator=(const ModelContext&);
};

class MechanicContext : public Referenced {
public:
  MechanicContext(const MechanicNode* mechanicNode) :
    mMechanicNode(mechanicNode)
  { }

  bool alloc()
  { return mMechanicNode->alloc(mLeafContext); }

  SharedPtr<const MechanicNode> mMechanicNode;
  LeafContext mLeafContext;

private:
  MechanicContext();
  MechanicContext(const MechanicContext&);
  MechanicContext& operator=(const MechanicContext&);
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

  LeafInstance(const LeafNode* leaf)
  { allocPorts(leaf); }
  virtual ~LeafInstance()
  { }

  LeafPortData* getPortData(const PortId& portId)
  {
    OpenFDMAssert(getLeafNode());
    unsigned index = getLeafNode()->getPortIndex(portId);
    // FIXME, is an error condition, not an assert???
    OpenFDMAssert(index < mPortData.size());
    return mPortData[index];
  }

  virtual const LeafNode* getLeafNode() const = 0;
  virtual void setPortValue(unsigned, PortValue*) const = 0;

  // Return true if this leaf directly depends on one of leafInstance outputs
  bool dependsOn(LeafInstance* leafInstance)
  {
    for (unsigned i = 0; i < mPortData.size(); ++i) {
      for (unsigned j = 0; j < leafInstance->mPortData.size(); ++j) {
        if (!mPortData[i]->isConnected(leafInstance->mPortData[j]))
          continue;
        PortId inPortId = getLeafNode()->getPortId(i);

        // FIXME, may be other concept:
        // make Model return a list of 'direct feedthrough ports'?
        for (unsigned k = 0; k < mPortData.size(); ++k) {
          PortId outPortId = getLeafNode()->getPortId(k);
          if (getLeafNode()->dependsOn(inPortId, outPortId))
            return true;
        }
      }
    }
    return false;
  }

  void print()
  {
    std::cout << "Leaf \"" << getLeafNode()->getName() << "\"" << std::endl;
    for (unsigned i = 0; i < mPortData.size(); ++i) {
      mPortData[i]->print();
    }
  }

  bool allocAndConnectProviderPortValues()
  {
    for (unsigned i = 0; i < getLeafNode()->getNumPorts(); ++i) {
      const ProviderPortInfo* providerPortInfo;
      providerPortInfo = getLeafNode()->getPort(i)->toProviderPortInfo();
      if (!providerPortInfo)
        continue;

      PortValue* portValue = providerPortInfo->newValue();
      setPortValue(i, portValue);
      
      // Also set the port value to all connected ports
      ProviderPortData* providerPortData = mPortData[i]->toProviderPortData();
      OpenFDMAssert(providerPortData);
      
      for (unsigned j = 0; j < providerPortData->_acceptorPortDataList.size(); ++j) {
        SharedPtr<AcceptorPortData> acceptorPortData;
        acceptorPortData = providerPortData->_acceptorPortDataList[j].lock();
        // Ok, happens for proxy ports, these still show up here
        if (!acceptorPortData)
          continue;
        SharedPtr<LeafInstance> leafInstance;
        leafInstance = acceptorPortData->getLeafInstance();
        OpenFDMAssert(leafInstance);
        
        OpenFDMAssert(acceptorPortData->getPortInfo());
        unsigned index = acceptorPortData->getPortInfo()->getIndex();
        leafInstance->setPortValue(index, portValue);
      }
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

  // List of port dependent info used to build up the connect info and
  // the sorted list of leafs.
  std::vector<SharedPtr<LeafPortData> > mPortData;
};

class ModelInstance : public LeafInstance {
public:
  ModelInstance(const Model* model) :
    LeafInstance(model),
    mModelContext(new ModelContext(model))
  { }

  virtual const Model* getLeafNode() const
  { return mModelContext->mModel; }

  virtual void setPortValue(unsigned i, PortValue* portValue) const
  { mModelContext->mLeafContext.mPortValueList.setPortValue(i, portValue); }

  bool alloc()
  { return mModelContext->alloc(); }
  void init()
  { mModelContext->init(); }
  void output()
  { mModelContext->output(); }
  void update()
  { mModelContext->update(); }

  SharedPtr<ModelContext> mModelContext;
};

class MechanicInstance : public LeafInstance {
public:
  MechanicInstance(const MechanicNode* mechanicNode) :
    LeafInstance(mechanicNode),
    mMechanicContext(new MechanicContext(mechanicNode))
  { }

  virtual const MechanicNode* getLeafNode() const
  { return mMechanicContext->mMechanicNode; }

  virtual void setPortValue(unsigned i, PortValue* portValue) const
  { mMechanicContext->mLeafContext.mPortValueList.setPortValue(i, portValue); }

  bool alloc()
  { return mMechanicContext->alloc(); }

  SharedPtr<MechanicContext> mMechanicContext;
};


class LeafInstanceCollector : public ConstNodeVisitor {
public:

  virtual void apply(const Node& node)
  { std::cerr << __PRETTY_FUNCTION__ << std::endl; }
  virtual void apply(const LeafNode& leaf)
  { std::cerr << __PRETTY_FUNCTION__ << std::endl; }
  virtual void apply(const LibraryNode& libraryNode)
  { std::cerr << __PRETTY_FUNCTION__ << std::endl; }

  // Aussen acceptor, innen provider
  virtual void apply(const GroupAcceptorNode& leaf)
  {
    OpenFDMAssert(leaf.getPort(0));
    PortId portId = leaf.getPortId(0);
    LeafInstance::ProviderPortData* pd;
    pd = new LeafInstance::ProviderPortData(0, leaf._groupInternalPort);
    _leafPortDataMap[getCurrentNodeId()][portId] = pd;
  }
  // Aussen provider, innen acceptor
  virtual void apply(const GroupProviderNode& leaf)
  {
    OpenFDMAssert(leaf.getPort(0));
    PortId portId = leaf.getPortId(0);
    LeafInstance::AcceptorPortData* ad;
    ad = new LeafInstance::AcceptorPortData(0, leaf._groupInternalPort);
    _leafPortDataMap[getCurrentNodeId()][portId] = ad;
  }

  void allocPortData(LeafInstance* leafInstance, const LeafNode& leaf)
  {
    // FIXME: move to LeafInstance??
    for (unsigned i = 0; i < leaf.getNumPorts(); ++i) {
      PortId portId = leaf.getPortId(i);
      LeafInstance::LeafPortData* portData = leafInstance->getPortData(portId);
      _leafPortDataMap[getCurrentNodeId()][portId] = portData;
    }
  }

  virtual void apply(const RootJoint& node)
  {
    // Need to stor the root nodes to build up the spanning tree for the
    // mechanical system here.
    MechanicInstance* mechanicInstance = new MechanicInstance(&node);
    _leafInstanceList.push_back(mechanicInstance);
//     _mechanicInstanceList.push_back(mechanicInstance);
    _rootJointInstanceList.push_back(mechanicInstance);
    allocPortData(mechanicInstance, node);
  }
  virtual void apply(const MechanicNode& node)
  {
    MechanicInstance* mechanicInstance = new MechanicInstance(&node);
    _leafInstanceList.push_back(mechanicInstance);
    _mechanicInstanceList.push_back(mechanicInstance);
    allocPortData(mechanicInstance, node);
  }
  virtual void apply(const Model& node)
  {
    ModelInstance* modelInstance = new ModelInstance(&node);
    _leafInstanceList.push_back(modelInstance);
    _modelInstanceList.push_back(modelInstance);
    allocPortData(modelInstance, node);
  }

  virtual void apply(const Group& group)
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

  typedef std::list<SharedPtr<ModelInstance> > ModelInstanceList;
  ModelInstanceList _modelInstanceList;
  typedef std::list<SharedPtr<MechanicInstance> > MechanicInstanceList;
  MechanicInstanceList _mechanicInstanceList;
//   typedef std::list<SharedPtr<RootJointInstance> > RootJointInstanceList;
  typedef std::list<SharedPtr<MechanicInstance> > RootJointInstanceList;
  RootJointInstanceList _rootJointInstanceList;

  ////////////////////////////////////////////////////////////////////////////
  // Used to map connections in groups ...
  typedef std::map<PortId, SharedPtr<LeafInstance::PortData> > NodePortDataMap;
  typedef std::map<Group::NodeId, NodePortDataMap> LeafPortDataMap;
  LeafPortDataMap _leafPortDataMap;


  // method to sort the leafs according to their dependency
  bool sortModelList()
  {
    ModelInstanceList sortedModelInstanceList;
    while (!_modelInstanceList.empty()) {
      SharedPtr<ModelInstance> modelInstance = _modelInstanceList.front();
      _modelInstanceList.pop_front();

      ModelInstanceList::iterator i;
      for (i = sortedModelInstanceList.begin();
           i != sortedModelInstanceList.end();
           ++i) {
        if (!(*i)->dependsOn(modelInstance))
          continue;
        sortedModelInstanceList.insert(i, modelInstance);
        break;
      }
      if (i == sortedModelInstanceList.end())
        sortedModelInstanceList.push_back(modelInstance);
    }
    _modelInstanceList.swap(sortedModelInstanceList);
  }

  typedef std::vector<SharedPtr<ModelInstance> > ModelContextList;

  bool
  getModelContextList(ModelContextList& modelContexts)
  {
    modelContexts.resize(0);

    ModelContextList modelContextList;
    modelContextList.reserve(_modelInstanceList.size());
    ModelInstanceList::const_iterator i;
    for (i = _modelInstanceList.begin(); i != _modelInstanceList.end(); ++i)
      modelContextList.push_back((*i));

    ModelContextList::const_iterator j;
    for (j = modelContextList.begin(); j != modelContextList.end(); ++j) {
      if (!(*j)->allocAndConnectProviderPortValues()) {
        Log(Schedule, Error) << "Could not alloc for model ... FIXME" << endl;
        return false;
      }
    }

    for (j = modelContextList.begin(); j != modelContextList.end(); ++j) {
      if (!(*j)->alloc()) {
        Log(Schedule, Error) << "Could not alloc for model ... FIXME" << endl;
        return false;
      }
    }

    // FIXME is here just for curiousity :)
    for (j = modelContextList.begin(); j != modelContextList.end(); ++j) {
      (*j)->init();
      (*j)->output();
    }

    modelContexts.swap(modelContextList);
    return true;
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

class PrintOutput : public Output::Callback {
public:
  virtual void setValue(real_type value)
  { std::cout << value << std::endl; }
};

int main()
{
  SharedPtr<Group> group = new Group("G0");
  Group::NodeId gain = group->addChild(new Gain("gain"));
  Group::NodeId integrator1 = group->addChild(new Integrator("I1"));
  Group::NodeId integrator2 = group->addChild(new Integrator("I2"));
  Group::NodeId output = group->addChild(new Output("O", new PrintOutput));
  Group::NodeId delay = group->addChild(new Delay("D"));
  Group::NodeId outputDelay = group->addChild(new Output("OD", new PrintOutput));

  group->connect(integrator1, "output", integrator2, "input");
  group->connect(integrator2, "output", gain, "input");
  group->connect(gain, "output", integrator1, "input");
  group->connect(integrator2, "output", output, "input");
  group->connect(gain, "output", delay, "input");
  group->connect(delay, "output", outputDelay, "input");

  //FIXME: broken naming
//   Group::NodeId groupOutputNode = group->addAcceptorPort();
  Group::NodeId groupOutputNode = group->addProviderPort();
  group->connect(integrator2, "output", groupOutputNode, "input");

  SharedPtr<Group> topGroup = new Group("G1");
  Group::NodeId child0 = topGroup->addChild(group);
  Group::NodeId child1 = topGroup->addChild(group);

  Group::NodeId output0 = topGroup->addChild(new Output("O2", new PrintOutput));
  topGroup->connect(child0, 0, output0, 0);
  Group::NodeId output1 = topGroup->addChild(new Output("O3", new PrintOutput));
  topGroup->connect(child1, 0, output1, 0);

  /////////////////////////////////////////////////

  LeafInstanceCollector nodeInstanceCollector;
  topGroup->accept(nodeInstanceCollector);
  
  nodeInstanceCollector.sortModelList();

  LeafInstanceCollector::ModelInstanceList::const_iterator i;
  for (i = nodeInstanceCollector._modelInstanceList.begin();
       i != nodeInstanceCollector._modelInstanceList.end();
       ++i) {
    (*i)->print();
  }

  LeafInstanceCollector::ModelContextList modelContextList;
  nodeInstanceCollector.getModelContextList(modelContextList);

  return 0;
}

// Kabelbaum <-> PortBundle ??? Original Kabelbaum == Cabel Bundle
// Oder Cable Set <-> Port Set???
