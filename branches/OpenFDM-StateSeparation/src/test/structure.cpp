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

#include <OpenFDM/SampleTime.h>

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
/// | NodeInstance |
/// ----------------
///             | |     --------
///             | ----->| Node |
///             |       --------
///             |       ---------------
///             ------->| NodeContext |
///                     ---------------
///
/// The NodeInstance is used to present the user a handle to the simulation
/// models runtime data. Each instance has an execution context and a pointer
/// to the actual node.
///
/// For model execution we have a list of ModelContexs and a list of
/// MechanicContexts. Both of them are derived from the NodeContext from above.
///
/// To connect the ports and propagate the port values over the NodeContexts
/// there must be a PortData like structure that is only built during simulation
/// model initialization.

/// A NodeInstance represents an effictive model node in a ready to run
/// System. You can access the Nodes Ports values for example.
/// This class is meant to show up in the user interface of this simulation.
class NodeInstance : public WeakReferenced {
public:
  virtual ~NodeInstance() {}

  /// The actual Node this NodeInstance stems from
  const Node& getNode() const
  { return getNodeContext().getNode(); }

  /// FIXME: put this to some global place
//   typedef std::vector<SharedPtr<const Node> > NodePath;
//   const NodePath& getNodePath() const { return mNodePath; }
//   NodePath mNodePath;

//   /// Set the sample times this node will run on
//   void setSampleTimeSet(const SampleTimeSet& sampleTimeSet)
//   { mSampleTimeSet = sampleTimeSet; }
//   /// Get the sample times this node will run on
//   const SampleTimeSet& getSampleTimeSet() const
//   { return mSampleTimeSet; }

protected:
  NodeInstance() {}

public: // FIXME????
  /// The node context that belongs to this instance.
  virtual NodeContext& getNodeContext() = 0;
  virtual const NodeContext& getNodeContext() const = 0;

private:
  NodeInstance(const NodeInstance&);
  NodeInstance& operator=(const NodeInstance&);

//   /// The sample times this node will run on
//   SampleTimeSet mSampleTimeSet;
};

// typedef std::vector<SharedPtr<NodeInstance> > NodeInstanceList;
// typedef std::vector<SharedPtr<const NodeInstance> > ConstNodeInstanceList;




////////////////////////////////////////////////////////////////////////////

/// This one will not show up in any execution list, but will be used
/// to fill NodeContext's for Node's that have nothing to execute,
/// should be reflected to the user of the simulation system. Group's
/// inputs ad outputs and their input and output models are such examples.
class GenericNodeContext : public NodeContext {
public:
  GenericNodeContext(const Node* node) :
    mNode(node)
  { }

  virtual const Node& getNode() const
  { return *mNode; }

private:
  GenericNodeContext();
  GenericNodeContext(const GenericNodeContext&);
  GenericNodeContext& operator=(const GenericNodeContext&);

  SharedPtr<const Node> mNode;
};

class GenericNodeInstance : public NodeInstance {
public:
  GenericNodeInstance(NodeContext* nodeContext) :
    mNodeContext(nodeContext)
  { }

  /// The node context that belongs to this instance.
  virtual NodeContext& getNodeContext()
  { return *mNodeContext; }
  virtual const NodeContext& getNodeContext() const
  { return *mNodeContext; }

  SharedPtr<NodeContext> mNodeContext;
};



//// This one is used to execute the simulation system
class ModelContext : public LeafContext {
public:
  ModelContext(const Model* model) :
    mModel(model)
  { }

  virtual const Model& getNode() const
  { return *mModel; }

  bool alloc()
  { return mModel->alloc(*this); }
  void init()
  { mModel->init(mDiscreteState, mContinousState); }
  void output()
  { mModel->output(mDiscreteState, mContinousState, mPortValueList); }
  void update()
  { mModel->update(mDiscreteState, mContinousState, mPortValueList); }

//   void derivative()
//   { mModel->derivative(mDiscreteState,
//                        mContinousState,
//                        mPortValueList,
//                        mContinousStateDerivative); }

  SharedPtr<const Model> mModel;

private:
  ModelContext();
  ModelContext(const ModelContext&);
  ModelContext& operator=(const ModelContext&);
};

class ModelInstance : public NodeInstance {
public:
  ModelInstance(const Model* model) :
    mModelContext(new ModelContext(model))
  { }

  virtual ModelContext& getNodeContext()
  { return *mModelContext; }
  virtual const ModelContext& getNodeContext() const
  { return *mModelContext; }

  SharedPtr<ModelContext> mModelContext;
};




class MechanicContext : public LeafContext {
public:
  MechanicContext(const MechanicNode* mechanicNode) :
    mMechanicNode(mechanicNode)
  { }

  virtual const MechanicNode& getNode() const
  { return *mMechanicNode; }

  bool alloc()
  { return mMechanicNode->alloc(*this); }
  void init()
  { mMechanicNode->init(mDiscreteState, mContinousState); }

  SharedPtr<const MechanicNode> mMechanicNode;

private:
  MechanicContext();
  MechanicContext(const MechanicContext&);
  MechanicContext& operator=(const MechanicContext&);
};

class MechanicInstance : public NodeInstance {
public:
  MechanicInstance(const MechanicNode* mechanicNode) :
    mMechanicContext(new MechanicContext(mechanicNode))
  { }

  virtual MechanicContext& getNodeContext()
  { return *mMechanicContext; }
  virtual const MechanicContext& getNodeContext() const
  { return *mMechanicContext; }

  SharedPtr<MechanicContext> mMechanicContext;
};






// Just here so that I do not care for intationation order for now ...
struct PortDataHelper {

  class PortDataList;

  struct LeafPortData;
  struct AcceptorPortData;
  struct ProviderPortData;
  struct ProxyPortData;

  class PortData : public WeakReferenced {
  public:
    PortData(PortDataList* portDataList) : mParentPortDataList(portDataList) { }
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

    SharedPtr<PortDataList> getParentPortDataList() const
    { return mParentPortDataList.lock(); }

    std::string getNodeName() const
    {
      SharedPtr<PortDataList> portDataList = getParentPortDataList();
      if (!portDataList)
        return std::string();
      SharedPtr<NodeInstance> nodeInstance = portDataList->mNodeInstance;
      if (!nodeInstance)
        return std::string();
      return nodeInstance->getNode().getName();
    }

    WeakPtr<PortDataList> mParentPortDataList;
  };

  class LeafPortData : public PortData {
  public:
    LeafPortData(PortDataList* portDataList, const PortInfo* portInfo) :
      PortData(portDataList),
      mPortInfo(portInfo)
    { }
    virtual ~LeafPortData() {}

    virtual LeafPortData* toLeafPortData()
    { return this; }
    
    const SharedPtr<const PortInfo>& getPortInfo() const
    { return mPortInfo; }

    virtual bool isConnected(LeafPortData*)
    { return false; }

    virtual void print()
    { }

  private:
    SharedPtr<const PortInfo> mPortInfo;
  };

  struct ProviderPortData : public LeafPortData {
    ProviderPortData(PortDataList* portDataList,
                     const ProviderPortInfo* providerPort) :
      LeafPortData(portDataList, providerPort),
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
        if (!acceptorPortData->getPortInfo())
          continue;
        std::cout << "    Node \"" << acceptorPortData->getNodeName()
                  << "\" Port \"" << acceptorPortData->getPortInfo()->getName()
                  << "\"" << std::endl;
      }
    }

    SharedPtr<const ProviderPortInfo> _providerPort;
    std::vector<WeakPtr<AcceptorPortData> > _acceptorPortDataList;
  };
  struct AcceptorPortData : public LeafPortData {
    AcceptorPortData(PortDataList* portDataList,
                     const AcceptorPortInfo* acceptorPort) :
      LeafPortData(portDataList, acceptorPort),
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
      if (!providerPortData->getPortInfo())
        return;
       std::cout << "    Node \"" << providerPortData->getNodeName()
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
    ProxyPortData(PortDataList* portDataList, PortData* portData) :
      PortData(portDataList)
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

  // Return true if this leaf directly depends on one of leafInstance outputs
  class PortDataList : public WeakReferenced {
  public:
    PortDataList(NodeInstance* nodeInstance) :
      mNodeInstance(nodeInstance)
    { }
    
    AcceptorPortData* newAcceptorPortData(const AcceptorPortInfo* acceptorPort)
    {
      AcceptorPortData* acceptorPortData;
      acceptorPortData = new AcceptorPortData(this, acceptorPort);
      mPortDataVector.push_back(acceptorPortData);
      return acceptorPortData;
    }
    ProviderPortData* newProviderPortData(const ProviderPortInfo* providerPort)
    {
      ProviderPortData* providerPortData;
      providerPortData = new ProviderPortData(this, providerPort);
      mPortDataVector.push_back(providerPortData);
      return providerPortData;
    }
    ProxyPortData* newProxyPortData(PortData* portData)
    {
      ProxyPortData* proxyPortData;
      proxyPortData = new ProxyPortData(this, portData);
      // FIXME
//       mPortDataVector.push_back(proxyPortData);
      return proxyPortData;
    }
    
    const Node& getNode() const
    { return mNodeInstance->getNode(); }

    void print()
    {
      for (unsigned i = 0; i < mPortDataVector.size(); ++i) {
        mPortDataVector[i]->print();
      }
    }
    
    bool allocAndConnectProviderPortValues()
    {
      for (unsigned i = 0; i < getNode().getNumPorts(); ++i) {
        const ProviderPortInfo* providerPortInfo;
        providerPortInfo = getNode().getPort(i)->toProviderPortInfo();
        if (!providerPortInfo)
          continue;
        
        PortValue* portValue = providerPortInfo->newValue();
        if (!portValue)
          continue;
        setPortValue(i, portValue);
        
        // Also set the port value to all connected ports
        ProviderPortData* providerPortData = mPortDataVector[i]->toProviderPortData();
        OpenFDMAssert(providerPortData);
        
        for (unsigned j = 0; j < providerPortData->_acceptorPortDataList.size(); ++j) {
          SharedPtr<AcceptorPortData> acceptorPortData;
          acceptorPortData = providerPortData->_acceptorPortDataList[j].lock();
          // Ok, happens for proxy ports, these still show up here
          if (!acceptorPortData)
            continue;
          
          SharedPtr<PortDataList> portDataList;
          portDataList = acceptorPortData->getParentPortDataList();
          OpenFDMAssert(portDataList);
          
          // FIXME, for now the GenericNodePorts do not have PortValues for
          // this reason.
          if (!acceptorPortData->getPortInfo())
            continue;
          unsigned index = acceptorPortData->getPortInfo()->getIndex();
          portDataList->setPortValue(index, portValue);
        }
      }
      return true;
    }
    void setPortValue(unsigned i, PortValue* portValue)
    { mNodeInstance->getNodeContext().getPortValueList().setPortValue(i, portValue); }

    // Return true if this leaf directly depends on one of leafInstance outputs
    bool dependsOn(PortDataList* portDataList)
    {
      for (unsigned i = 0; i < mPortDataVector.size(); ++i) {
        for (unsigned j = 0; j < portDataList->mPortDataVector.size(); ++j) {
          if (!mPortDataVector[i]->isConnected(portDataList->mPortDataVector[j]))
            continue;
          PortId inPortId = getNode().getPortId(i);
          
          // FIXME, may be other concept:
          // make Model return a list of 'direct feedthrough ports'?
          for (unsigned k = 0; k < mPortDataVector.size(); ++k) {
            PortId outPortId = getNode().getPortId(k);
            // FIXME ugly
            const Node* node = &(mNodeInstance->getNode());
            const LeafNode* leafNode = dynamic_cast<const LeafNode*>(node);
            if (leafNode && leafNode->dependsOn(inPortId, outPortId))
              return true;
          }
        }
      }
      return false;
    }

    /// The vector of per port connect information
    typedef std::vector<SharedPtr<LeafPortData> > PortDataVector;
    PortDataVector mPortDataVector;
    
    /// The NodeInstance having some way to reference the
    /// PortValues to the current connect information.
    SharedPtr<NodeInstance> mNodeInstance;
  };
};

class LeafInstanceCollector : public ConstNodeVisitor {
public:

  virtual void apply(const Node& node)
  { std::cerr << __PRETTY_FUNCTION__ << std::endl; }
  virtual void apply(const LeafNode& leaf)
  { std::cerr << __PRETTY_FUNCTION__ << std::endl; }
  virtual void apply(const LibraryNode& libraryNode)
  { std::cerr << __PRETTY_FUNCTION__ << std::endl; }

  PortDataHelper::PortDataList* buildGenericNodeContext(const Node& node)
  {
    GenericNodeContext* genericNodeContext = new GenericNodeContext(&node);
    GenericNodeInstance* genericNodeInstance;
    genericNodeInstance = new GenericNodeInstance(genericNodeContext);
    _nodeInstanceList.push_back(genericNodeInstance);
    PortDataHelper::PortDataList* portDataList = new PortDataHelper::PortDataList(genericNodeInstance);
    _portDataListMap[_nodeInstanceList.back()] = portDataList;

    return portDataList;
  }

  // Aussen acceptor, innen provider
  virtual void apply(const GroupAcceptorNode& leaf)
  {
    PortDataHelper::PortDataList* portDataList = buildGenericNodeContext(leaf);

    OpenFDMAssert(leaf.getPort(0));
    PortId portId = leaf.getPortId(0);

    PortDataHelper::ProviderPortData* providerPortData;
//     providerPortData = portDataList->newProviderPortData(leaf._groupInternalPort);
    providerPortData = portDataList->newProviderPortData(0 /*FIXME*/);
    _leafPortDataMap[getCurrentNodeId()][portId] = providerPortData;
  }
  // Aussen provider, innen acceptor
  virtual void apply(const GroupProviderNode& leaf)
  {
    PortDataHelper::PortDataList* portDataList = buildGenericNodeContext(leaf);

    OpenFDMAssert(leaf.getPort(0));
    PortId portId = leaf.getPortId(0);

    PortDataHelper::AcceptorPortData* acceptorPortData;
//     acceptorPortData = portDataList->newAcceptorPortData(leaf._groupInternalPort);
    acceptorPortData = portDataList->newAcceptorPortData(0 /*FIXME*/);
    _leafPortDataMap[getCurrentNodeId()][portId] = acceptorPortData;
  }

  void allocPortData(NodeInstance* leafInstance, const LeafNode& leaf)
  {
    PortDataHelper::PortDataList* portDataList = new PortDataHelper::PortDataList(leafInstance);
    _portDataListMap[SharedPtr<NodeInstance>(leafInstance)] = portDataList;

    // FIXME: move to LeafInstance??
    for (unsigned i = 0; i < leaf.getNumPorts(); ++i) {
      SharedPtr<const PortInfo> port = leaf.getPort(i);
      const ProviderPortInfo* providerPort = port->toProviderPortInfo();
      if (providerPort) {
        PortDataHelper::ProviderPortData* providerPortData;
        providerPortData = portDataList->newProviderPortData(providerPort);

        PortId portId = leaf.getPortId(i);
        _leafPortDataMap[getCurrentNodeId()][portId] = providerPortData;
      }
      const AcceptorPortInfo* acceptorPort = port->toAcceptorPortInfo();
      if (acceptorPort) {
        PortDataHelper::AcceptorPortData* acceptorPortData;
        acceptorPortData = portDataList->newAcceptorPortData(acceptorPort);

        PortId portId = leaf.getPortId(i);
        _leafPortDataMap[getCurrentNodeId()][portId] = acceptorPortData;
      }
    }
  }

  virtual void apply(const RootJoint& node)
  {
    // Need to stor the root nodes to build up the spanning tree for the
    // mechanical system here.
    MechanicInstance* mechanicInstance = new MechanicInstance(&node);
    _nodeInstanceList.push_back(mechanicInstance);
//     _mechanicInstanceList.push_back(mechanicInstance);
    _rootJointInstanceList.push_back(mechanicInstance);
    allocPortData(mechanicInstance, node);
  }
  virtual void apply(const MechanicNode& node)
  {
    MechanicInstance* mechanicInstance = new MechanicInstance(&node);
    _nodeInstanceList.push_back(mechanicInstance);
    _mechanicInstanceList.push_back(mechanicInstance);
    allocPortData(mechanicInstance, node);
  }
  virtual void apply(const Model& node)
  {
    ModelInstance* modelInstance = new ModelInstance(&node);
    _nodeInstanceList.push_back(modelInstance);
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

    PortDataHelper::PortDataList* portDataList = buildGenericNodeContext(group);

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
          portDataList->newProxyPortData(_leafPortDataMap[nodeId].begin()->second);
    }

    parentLeafPortDataMap.swap(_leafPortDataMap);
  }

  ////////////////////////////////////////////////////////////////////////////
  // The final list of Nodes we have in the simulation system
  typedef std::list<SharedPtr<NodeInstance> > NodeInstanceList;
  NodeInstanceList _nodeInstanceList;


  // The Models list, worthwhile for sorting
  typedef std::list<SharedPtr<ModelInstance> > ModelInstanceList;
  ModelInstanceList _modelInstanceList;
  typedef std::list<SharedPtr<MechanicInstance> > MechanicInstanceList;
  MechanicInstanceList _mechanicInstanceList;
//   typedef std::list<SharedPtr<RootJointInstance> > RootJointInstanceList;
  typedef std::list<SharedPtr<MechanicInstance> > RootJointInstanceList;
  RootJointInstanceList _rootJointInstanceList;

  ////////////////////////////////////////////////////////////////////////////
  // Used to map connections in groups ...
  typedef std::map<PortId, SharedPtr<PortDataHelper::PortData> > NodePortDataMap;
  typedef std::map<Group::NodeId, NodePortDataMap> LeafPortDataMap;
  LeafPortDataMap _leafPortDataMap;
  // Just to hold references to all mort data lists we have in the
  // simulation system. They are just needed during traversal for connect
  // information and to distribute port value pointers.
  typedef std::map<SharedPtr<NodeInstance>,SharedPtr<PortDataHelper::PortDataList> > PortDataListMap;
  PortDataListMap _portDataListMap;


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

        PortDataHelper::PortDataList* portDataListMI
          = _portDataListMap[modelInstance];
        PortDataHelper::PortDataList* portDataListI = _portDataListMap[*i];

        if (!portDataListI->dependsOn(portDataListMI))
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
      PortDataHelper::PortDataList* portDataList = _portDataListMap[*j];
      if (!portDataList)
        continue;
      if (!portDataList->allocAndConnectProviderPortValues()) {
        Log(Schedule, Error) << "Could not alloc for model ... FIXME" << endl;
        return false;
      }
    }

    for (j = modelContextList.begin(); j != modelContextList.end(); ++j) {
      if (!(*j)->getNodeContext().alloc()) {
        Log(Schedule, Error) << "Could not alloc for model ... FIXME" << endl;
        return false;
      }
    }

    // FIXME is here just for curiousity :)
    for (j = modelContextList.begin(); j != modelContextList.end(); ++j) {
      (*j)->getNodeContext().init();
      (*j)->getNodeContext().output();
    }

    modelContexts.swap(modelContextList);
    return true;
  }

  void print()
  {
    ModelInstanceList::iterator i;
    for (i = _modelInstanceList.begin(); i != _modelInstanceList.end(); ++i) {
      std::cout << "Model \"" << (*i)->getNode().getName() << "\"" << std::endl;
      PortDataHelper::PortDataList* portDataList = _portDataListMap[*i];
      if (!portDataList)
        continue;
      portDataList->print();
    }
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

  nodeInstanceCollector.print();

  LeafInstanceCollector::ModelContextList modelContextList;
  nodeInstanceCollector.getModelContextList(modelContextList);

  return 0;
}

// Kabelbaum <-> PortBundle ??? Original Kabelbaum == Cabel Bundle
// Oder Cable Set <-> Port Set???

