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

typedef std::vector<SharedPtr<const Node> > NodePath;

/// A NodeInstance represents an effective model node in a ready to run
/// System. You can access the Nodes Ports values for example.
/// This class is meant to show up in the user interface of this simulation.
class NodeInstance : public WeakReferenced {
public:
  NodeInstance(const NodePath& nodePath) :
    mNodePath(nodePath)
  { }
  virtual ~NodeInstance()
  { }

  /// The actual Node this NodeInstance stems from
  const Node& getNode() const
  { return getNodeContext().getNode(); }

  const NodePath& getNodePath() const { return mNodePath; }

//   /// Set the sample times this node will run on
//   void setSampleTimeSet(const SampleTimeSet& sampleTimeSet)
//   { mSampleTimeSet = sampleTimeSet; }
//   /// Get the sample times this node will run on
//   const SampleTimeSet& getSampleTimeSet() const
//   { return mSampleTimeSet; }

  PortValueList& getPortValueList()
  { return getNodeContext().getPortValueList(); }
  const PortValueList& getPortValueList() const
  { return getNodeContext().getPortValueList(); }

  std::string getNodeNamePath() const
  {
    if (mNodePath.empty())
      return std::string();
    std::string path = mNodePath.front()->getName();
    NodePath::const_iterator i = mNodePath.begin();
    if (i != mNodePath.end()) {
      for (++i; i != mNodePath.end(); ++i) {
        path += '/';
        path += (*i)->getName();
      }
    }
    return path;
  }

protected:
  NodeInstance() {}

  /// The node context that belongs to this instance.
  virtual NodeContext& getNodeContext() = 0;
  virtual const NodeContext& getNodeContext() const = 0;

private:
  NodeInstance(const NodeInstance&);
  NodeInstance& operator=(const NodeInstance&);

//   /// The sample times this node will run on
//   SampleTimeSet mSampleTimeSet;

  NodePath mNodePath;
};

typedef std::list<SharedPtr<NodeInstance> > NodeInstanceList;


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
  GenericNodeInstance(const NodePath& nodePath, const Node* node) :
    NodeInstance(nodePath),
    mNodeContext(new GenericNodeContext(node))
  { }

protected:
  /// The node context that belongs to this instance.
  virtual NodeContext& getNodeContext()
  { return *mNodeContext; }
  virtual const NodeContext& getNodeContext() const
  { return *mNodeContext; }

private:
  SharedPtr<NodeContext> mNodeContext;
};

class Task;
class DiscreteTask;
class ContinousTask;

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
  void output(const Task&)
  { mModel->output(mDiscreteState, mContinousState, mPortValueList); }
  void update(const DiscreteTask&)
  { mModel->update(mDiscreteState, mContinousState, mPortValueList); }

//   void derivative()
//   { mModel->derivative(mDiscreteState,
//                        mContinousState,
//                        mPortValueList,
//                        mContinousStateDerivative); }

  // Return true if this leaf directly depends on one of leafInstance outputs
  bool dependsOn(const ModelContext* modelContext) const
  {
    unsigned numPorts = mModel->getNumPorts();
    for (unsigned i = 0; i < numPorts; ++i) {
      const AcceptorPortInfo* acceptorPortInfo;
      acceptorPortInfo = mModel->getPort(i)->toAcceptorPortInfo();
      if (!acceptorPortInfo)
        continue;
      if (!acceptorPortInfo->getDirectInput())
        continue;
      const PortValue* portValue = getPortValueList().getPortValue(i);
      if (!portValue)
        continue;
      unsigned otherNumPorts = modelContext->mModel->getNumPorts();
      for (unsigned j = 0; j < otherNumPorts; ++j) {
        if (!modelContext->mModel->getPort(j)->toProviderPortInfo())
          continue;

        const PortValue* otherPortValue;
        otherPortValue = modelContext->getPortValueList().getPortValue(j);
        if (portValue != otherPortValue)
          continue;

        return true;
      }
    }
    return false;
  }

private:
  ModelContext();
  ModelContext(const ModelContext&);
  ModelContext& operator=(const ModelContext&);

  SharedPtr<const Model> mModel;
};

class ModelContextList : public std::list<SharedPtr<ModelContext> > {
public:
  typedef std::list<SharedPtr<ModelContext> > list_type;

  bool alloc() const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      if (!(*i)->alloc())
        return false;
    return true;
  }
  void init() const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->init();
  }
  void output(const Task& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->output(task);
  }
  void update(const DiscreteTask& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->update(task);
  }
};

class ModelInstance : public NodeInstance {
public:
  ModelInstance(const NodePath& nodePath, const Model* model) :
    NodeInstance(nodePath),
    mModelContext(new ModelContext(model))
  { }

  // Return true if this leaf directly depends on one of leafInstance outputs
  bool dependsOn(const ModelInstance* modelInstance) const
  { return mModelContext->dependsOn(modelInstance->mModelContext); }

// protected: // FIXME
  virtual ModelContext& getNodeContext()
  { return *mModelContext; }
  virtual const ModelContext& getNodeContext() const
  { return *mModelContext; }

private:
  SharedPtr<ModelContext> mModelContext;
};

typedef std::list<SharedPtr<ModelInstance> > ModelInstanceList;



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

  void velocities(const ContinousTask&)
  { mMechanicNode->velocity(mContinousState, mPortValueList); }
  void articulation(const ContinousTask&)
  { mMechanicNode->articulation(mContinousState, mPortValueList); }
  void accelerations(const ContinousTask&)
  { }

//   virtual void derivative(const ContinousStateValueVector&,
//                           const PortValueList&,
//                           ContinousStateValueVector&) const
 
//   void outputVelocities()
//   { }
//   void outputAcceperation()
//   { }

//   void update()
//   { }

  SharedPtr<const MechanicNode> mMechanicNode;

private:
  MechanicContext();
  MechanicContext(const MechanicContext&);
  MechanicContext& operator=(const MechanicContext&);
};

class MechanicContextList : public std::list<SharedPtr<MechanicContext> > {
public:
  typedef std::list<SharedPtr<MechanicContext> > list_type;

  bool alloc() const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      if (!(*i)->alloc())
        return false;
    return true;
  }
  void init() const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->init();
  }
  void velocities(const ContinousTask& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->velocities(task);
  }
  void articulation(const ContinousTask& task) const
  {
    // Note that this list is traversed from the mechanic leafs to the root
    for (list_type::const_reverse_iterator i = rbegin(); i != rend(); ++i)
      (*i)->articulation(task);
  }
  void accelerations(const ContinousTask& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->accelerations(task);
  }
};

class MechanicInstance : public NodeInstance {
public:
  MechanicInstance(const NodePath& nodePath,
                   const MechanicNode* mechanicNode) :
    NodeInstance(nodePath),
    mMechanicContext(new MechanicContext(mechanicNode))
  { }

protected:
  virtual MechanicContext& getNodeContext()
  { return *mMechanicContext; }
  virtual const MechanicContext& getNodeContext() const
  { return *mMechanicContext; }

private:
  SharedPtr<MechanicContext> mMechanicContext;
};

typedef std::list<SharedPtr<MechanicInstance> > MechanicInstanceList;





// Just here so that I do not care for intationation order for now ...
struct PortDataHelper {

  class PortDataList;

  struct AcceptorPortData;
  struct ProviderPortData;
  struct ProxyAcceptorPortData;
  struct ProxyProviderPortData;

  struct PortData : public WeakReferenced {
  public:
    PortData(PortDataList* portDataList, const PortInfo* portInfo = 0) :
      mParentPortDataList(portDataList),
      mPortInfo(portInfo)
    { }
    virtual ~PortData()
    { }
    virtual AcceptorPortData* toAcceptorPortData()
    { return 0; }
    virtual ProviderPortData* toProviderPortData()
    { return 0; }
    virtual ProxyAcceptorPortData* toProxyAcceptorPortData()
    { return 0; }
    virtual ProxyProviderPortData* toProxyProviderPortData()
    { return 0; }
    virtual bool connect(PortData*) = 0;

    SharedPtr<PortDataList> getParentPortDataList() const
    { return mParentPortDataList.lock(); }
    SharedPtr<NodeInstance> getNodeInstance() const
    {
      SharedPtr<PortDataList> portDataList = getParentPortDataList();
      if (!portDataList)
        return 0;
      return portDataList->mNodeInstance;
    }

    const SharedPtr<const PortInfo>& getPortInfo() const
    { return mPortInfo; }

    void setLocalPortValue(PortValue* portValue)
    {
      if (!getPortInfo())
        return;
      SharedPtr<NodeInstance> nodeInstance = getNodeInstance();
      if (!nodeInstance)
        return;
      unsigned index = getPortInfo()->getIndex();
      nodeInstance->getPortValueList().setPortValue(index, portValue);
    }

  private:
    WeakPtr<PortDataList> mParentPortDataList;
    SharedPtr<const PortInfo> mPortInfo;
  };

  struct ProviderPortData : public PortData {
    ProviderPortData(PortDataList* portDataList,
                     const ProviderPortInfo* providerPort) :
      PortData(portDataList, providerPort),
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

    virtual void createPortValue()
    {
      PortValue* portValue = _providerPort->newValue();
      if (!portValue)
        return;
      setPortValue(portValue);
    }

    void setPortValue(PortValue* portValue)
    {
      setLocalPortValue(portValue);
      for (unsigned i = 0; i < _acceptorPortDataList.size(); ++i)
        _acceptorPortDataList[i]->setPortValue(portValue);
    }

    SharedPtr<const ProviderPortInfo> _providerPort;
    std::vector<SharedPtr<AcceptorPortData> > _acceptorPortDataList;
  };
  struct AcceptorPortData : public PortData {
    AcceptorPortData(PortDataList* portDataList,
                     const AcceptorPortInfo* acceptorPort) :
      PortData(portDataList, acceptorPort),
      _acceptorPort(acceptorPort)
    { }
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

    virtual void setPortValue(PortValue* portValue)
    {
      setLocalPortValue(portValue);
    }

    SharedPtr<const AcceptorPortInfo> _acceptorPort;
    WeakPtr<ProviderPortData> _providerPortData;
  };
  struct ProxyAcceptorPortData : public AcceptorPortData {
  public:
    ProxyAcceptorPortData(PortDataList* portDataList,
                          const AcceptorPortInfo* acceptorPortInfo) :
      AcceptorPortData(portDataList, acceptorPortInfo)
    { }
    virtual ProxyAcceptorPortData* toProxyAcceptorPortData()
    { return this; }
    virtual void setPortValue(PortValue* portValue)
    {
      setLocalPortValue(portValue);
      mProxyProviderPortData->setPortValue(portValue);
    }
    SharedPtr<ProxyProviderPortData> mProxyProviderPortData;
  };
  struct ProxyProviderPortData : public ProviderPortData {
  public:
    ProxyProviderPortData(PortDataList* portDataList,
                          const ProviderPortInfo* providerPortInfo) :
      ProviderPortData(portDataList, providerPortInfo)
    { }
    virtual ProxyProviderPortData* toProxyProviderPortData()
    { return this; }
    void setProxyAcceptorPortData(ProxyAcceptorPortData* proxyAcceptorPortData)
    { proxyAcceptorPortData->mProxyProviderPortData = this; }

    // FIXME
    virtual void createPortValue()
    { }
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
    ProxyAcceptorPortData* newProxyAcceptorPortData(const AcceptorPortInfo* acceptorPort)
    {
      ProxyAcceptorPortData* acceptorPortData;
      acceptorPortData = new ProxyAcceptorPortData(this, acceptorPort);
      mPortDataVector.push_back(acceptorPortData);
      return acceptorPortData;
    }
    ProxyProviderPortData* newProxyProviderPortData(const ProviderPortInfo* providerPort)
    {
      ProxyProviderPortData* providerPortData;
      providerPortData = new ProxyProviderPortData(this, providerPort);
      mPortDataVector.push_back(providerPortData);
      return providerPortData;
    }
    
    bool allocAndConnectProviderPortValues()
    {
      // FIXME: move that into the PortData stuff
      for (unsigned i = 0; i < mPortDataVector.size(); ++i) {
        // Also set the port value to all connected ports
        ProviderPortData* providerPortData;
        providerPortData = mPortDataVector[i]->toProviderPortData();
        if (!providerPortData)
          continue;
        providerPortData->createPortValue();
      }
      return true;
    }

    /// The vector of per port connect information
    typedef std::vector<SharedPtr<PortData> > PortDataVector;
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
    GenericNodeInstance* genericNodeInstance;
    genericNodeInstance = new GenericNodeInstance(mNodePath, &node);
    _nodeInstanceList.push_back(genericNodeInstance);
    PortDataHelper::PortDataList* portDataList = new PortDataHelper::PortDataList(genericNodeInstance);
    _portDataListList.push_back(portDataList);

    return portDataList;
  }

  // Aussen acceptor, innen provider
  virtual void apply(const GroupAcceptorNode& leaf)
  {
    PortDataHelper::PortDataList* portDataList = buildGenericNodeContext(leaf);

    OpenFDMAssert(leaf.getPort(0));
    PortId portId = leaf.getPortId(0);

    PortDataHelper::ProviderPortData* providerPortData;
    providerPortData = portDataList->newProxyProviderPortData(leaf._groupInternalPort);
    getCurrentNodePortDataMap()[portId] = providerPortData;
  }
  // Aussen provider, innen acceptor
  virtual void apply(const GroupProviderNode& leaf)
  {
    PortDataHelper::PortDataList* portDataList = buildGenericNodeContext(leaf);

    OpenFDMAssert(leaf.getPort(0));
    PortId portId = leaf.getPortId(0);

    PortDataHelper::AcceptorPortData* acceptorPortData;
    acceptorPortData = portDataList->newProxyAcceptorPortData(leaf._groupInternalPort);
    getCurrentNodePortDataMap()[portId] = acceptorPortData;
  }

  void allocPortData(NodeInstance* leafInstance, const LeafNode& leaf)
  {
    PortDataHelper::PortDataList* portDataList = new PortDataHelper::PortDataList(leafInstance);
    _portDataListList.push_back(portDataList);

    // FIXME: move to LeafInstance??
    for (unsigned i = 0; i < leaf.getNumPorts(); ++i) {
      SharedPtr<const PortInfo> port = leaf.getPort(i);
      const ProviderPortInfo* providerPort = port->toProviderPortInfo();
      if (providerPort) {
        PortDataHelper::ProviderPortData* providerPortData;
        providerPortData = portDataList->newProviderPortData(providerPort);

        PortId portId = leaf.getPortId(i);
        getCurrentNodePortDataMap()[portId] = providerPortData;
      }
      const AcceptorPortInfo* acceptorPort = port->toAcceptorPortInfo();
      if (acceptorPort) {
        PortDataHelper::AcceptorPortData* acceptorPortData;
        acceptorPortData = portDataList->newAcceptorPortData(acceptorPort);

        PortId portId = leaf.getPortId(i);
        getCurrentNodePortDataMap()[portId] = acceptorPortData;
      }
    }
  }

  virtual void apply(const RootJoint& node)
  {
    // Need to stor the root nodes to build up the spanning tree for the
    // mechanical system here.
    MechanicInstance* mechanicInstance = new MechanicInstance(mNodePath, &node);
    _nodeInstanceList.push_back(mechanicInstance);
//     _mechanicInstanceList.push_back(mechanicInstance);
    _rootJointInstanceList.push_back(mechanicInstance);
    allocPortData(mechanicInstance, node);
  }
  virtual void apply(const MechanicNode& node)
  {
    MechanicInstance* mechanicInstance = new MechanicInstance(mNodePath, &node);
    _nodeInstanceList.push_back(mechanicInstance);
    _mechanicInstanceList.push_back(mechanicInstance);
    allocPortData(mechanicInstance, node);
  }
  virtual void apply(const Model& node)
  {
    ModelInstance* modelInstance = new ModelInstance(mNodePath, &node);
    _nodeInstanceList.push_back(modelInstance);
    _modelInstanceList.push_back(modelInstance);
    allocPortData(modelInstance, node);
  }

  virtual void apply(const Group& group)
  {
    // Prepare a new leaf map for the child group
    PortDataMap parentPortDataMap;
    parentPortDataMap.swap(_portDataMap);

    // Walk the children
    for (unsigned i = 0; i < group.getNumChildren(); ++i) {
      pushNodeId(i);
      mNodePath.push_back(group.getChild(i));
      group.getChild(i)->accept(*this);
      mNodePath.pop_back();
      popNodeId();
    }

    // Apply the group internal connections to the instances
    unsigned numConnects = group.getNumConnects();
    for (unsigned i = 0; i < numConnects; ++i) {
      unsigned acceptorNodeIndex = group.getConnectAcceptorNodeIndex(i);
      unsigned providerNodeIndex = group.getConnectProviderNodeIndex(i);

      if (acceptorNodeIndex == ~0u) {
        std::cerr << "Cannot find acceptor node from nodeId" << std::endl;
        continue;
      }
      if (providerNodeIndex == ~0u) {
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
                  << group.getChild(acceptorNodeIndex)->getName() << std::endl;
        continue;
      }
      if (!providerPort) {
        std::cerr << "Cannot find provider Port data node "
                  << group.getChild(providerNodeIndex)->getName() << std::endl;
        continue;
      }

      if (!_portDataMap[acceptorNodeIndex][acceptorPortId]->
          connect(_portDataMap[providerNodeIndex][providerPortId]))
        std::cerr << "Cannot connect????" << std::endl;
    }

    PortDataHelper::PortDataList* portDataList = buildGenericNodeContext(group);

    parentPortDataMap.swap(_portDataMap);
    // Ok, some nameing niceness
    PortDataMap childrenPortDataMap;
    childrenPortDataMap.swap(parentPortDataMap);

    // add group connect routings
    // merge child list into the global list of instances
    for (unsigned i = 0; i < group.getNumPorts(); ++i) {
      PortId portId = group.getPortId(i);
      unsigned nodeIndex = group.getGroupPortNodeIndex(portId);
      if (childrenPortDataMap[nodeIndex].empty()) {
        // FIXME, is this an internal error ???
        std::cerr << "Hmm, cannot find GroupPortNode for external port "
                  << i << std::endl;
        continue;
      }

      PortDataHelper::PortData* portData = childrenPortDataMap[nodeIndex].begin()->second;
      if (portData->toProxyAcceptorPortData()) {
        PortDataHelper::ProxyAcceptorPortData* proxyAcceptorPortData;
        proxyAcceptorPortData = portData->toProxyAcceptorPortData();

        const ProviderPortInfo* providerPortInfo;
        providerPortInfo = group.getPort(i)->toProviderPortInfo();
        OpenFDMAssert(providerPortInfo);

        PortDataHelper::ProxyProviderPortData* proxyProviderPortData;
        proxyProviderPortData =
          portDataList->newProxyProviderPortData(providerPortInfo);

        proxyProviderPortData->setProxyAcceptorPortData(proxyAcceptorPortData);

        getCurrentNodePortDataMap()[portId] = proxyProviderPortData;

      } else if (portData->toProxyProviderPortData()) {
        PortDataHelper::ProxyProviderPortData* proxyProviderPortData;
        proxyProviderPortData = portData->toProxyProviderPortData();

        const AcceptorPortInfo* acceptorPortInfo;
        acceptorPortInfo = group.getPort(i)->toAcceptorPortInfo();
        OpenFDMAssert(acceptorPortInfo);

        PortDataHelper::ProxyAcceptorPortData* proxyAcceptorPortData;
        proxyAcceptorPortData =
          portDataList->newProxyAcceptorPortData(acceptorPortInfo);

        proxyProviderPortData->setProxyAcceptorPortData(proxyAcceptorPortData);

        getCurrentNodePortDataMap()[portId] = proxyAcceptorPortData;

      } else {
        OpenFDMAssert(false);
      }
    }
  }

  ////////////////////////////////////////////////////////////////////////////
  // The final list of Nodes we have in the simulation system
  NodeInstanceList _nodeInstanceList;


  // The Models list, worthwhile for sorting
  ModelInstanceList _modelInstanceList;
  // The mechanical system list, also for sorting
  MechanicInstanceList _mechanicInstanceList;
  // The list of root nodes in the mechanical system. Will be a starting point
  // for sorting the tree of mechanical models downwards
  typedef MechanicInstanceList RootJointInstanceList;
  RootJointInstanceList _rootJointInstanceList;

  ////////////////////////////////////////////////////////////////////////////
  // Used to map connections in groups ...
  typedef std::map<PortId, SharedPtr<PortDataHelper::PortData> > NodePortDataMap;
  typedef std::map<unsigned, NodePortDataMap> PortDataMap;
  PortDataMap _portDataMap;
  // Just to hold references to all mort data lists we have in the
  // simulation system. They are just needed during traversal for connect
  // information and to distribute port value pointers.
  typedef std::list<SharedPtr<PortDataHelper::PortDataList> > PortDataListList;
  PortDataListList _portDataListList;

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

        // Something already sorted in depends on modelInstance,
        // so schedule that new thing just before.
        Log(Schedule, Info)
          << "Inserting Model \"" << modelInstance->getNodeNamePath()
          << "\" before Model \"" << (*i)->getNodeNamePath()
          << "\"" << std::endl;
        i = sortedModelInstanceList.insert(i, modelInstance);
        break;
      }
      if (i == sortedModelInstanceList.end()) {
        // nothing found so far that depends on model instance.
        // So put it at the end.
        Log(Schedule, Info)
          << "Appending Model \"" << modelInstance->getNodeNamePath()
          << "\"" << std::endl;

        sortedModelInstanceList.push_back(modelInstance);
      } else {
        // If it cannot be put at the end, check if modelInstance depends
        // on any model that is already scheduled behind to detect cyclic loops.
        for (; i != sortedModelInstanceList.end(); ++i) {
          if (!modelInstance->dependsOn(*i))
            continue;
          Log(Schedule,Error)
            << "Detected cyclic loop: Model \""
            << modelInstance->getNodeNamePath() << "\" depends on Model \""
            << (*i)->getNodeNamePath() << "\"" << std::endl;
          return false;
        }
      }
    }
    _modelInstanceList.swap(sortedModelInstanceList);
    return true;
  }

  bool
  allocPortValues()
  {
    PortDataListList::const_iterator i;
    for (i = _portDataListList.begin(); i != _portDataListList.end(); ++i) {
      if (!(*i)->allocAndConnectProviderPortValues()) {
        Log(Schedule, Error) << "Could not alloc for model ... FIXME" << endl;
        return false;
      }
    }
    return true;
  }

  bool
  getModelContextList(ModelInstanceList& modelContexts)
  {
    modelContexts.resize(0);

    ModelInstanceList modelContextList;
    ModelInstanceList::const_iterator i;
    for (i = _modelInstanceList.begin(); i != _modelInstanceList.end(); ++i)
      modelContextList.push_back((*i));

    ModelInstanceList::const_iterator j;
    for (j = modelContextList.begin(); j != modelContextList.end(); ++j) {
      if (!(*j)->getNodeContext().alloc()) {
        Log(Schedule, Error) << "Could not alloc for model ... FIXME" << endl;
        return false;
      }
    }

    // FIXME is here just for curiousity :)
    for (j = modelContextList.begin(); j != modelContextList.end(); ++j) {
      (*j)->getNodeContext().init();
      (*j)->getNodeContext().output(*reinterpret_cast<Task*>(0));
    }

    modelContexts.swap(modelContextList);
    return true;
  }

  void pushNodeId(unsigned index)
  { mNodeIndexStack.push_back(index); }
  void popNodeId()
  { mNodeIndexStack.pop_back(); }

  NodePortDataMap& getCurrentNodePortDataMap()
  {
    OpenFDMAssert(!mNodeIndexStack.empty());
    return _portDataMap[mNodeIndexStack.back()];
  }

private:
  typedef std::list<unsigned> NodeIndexStack;
  NodeIndexStack mNodeIndexStack;

  NodePath mNodePath;
};




class Task : public Referenced {
public:

  void setTime(const real_type& time)
  { mTime = time; }
  const real_type& getTime() const
  { return mTime; }

private:
  real_type mTime;
};

class ContinousTask : public Task {
public:

  void output() const
  {
    // The model outputs before mechanical state propagation
    mModelContextList[0].output(*this);
    // Now the mechanical state propagation
    mMechanicContextList.velocities(*this);
    // The model outputs before mechanical force propagation
    mModelContextList[1].output(*this);
    // Now the mechanical force propagation
    mMechanicContextList.articulation(*this);
    // The model outputs before mechanical acceleration propagation
    mModelContextList[2].output(*this);
    // Now the mechanical acceleration propagation
    mMechanicContextList.accelerations(*this);
    // The model outputs past mechanical acceleration propagation
    mModelContextList[3].output(*this);
  }

  void derivative() const
  {
    // FIXME
//     for (unsigned i = 0; i < 4; ++i)
//       mModelContextList[i].derivative(*this);
//     mMechanicContextList.derivative(*this);
  }

  ModelContextList mModelContextList[4];
  MechanicContextList mMechanicContextList;
};

class DiscreteTask : public Task {
public:
  DiscreteTask(const real_type& stepsize) : mStepsize(stepsize)
  { }

  const real_type& getStepsize() const
  { return mStepsize; }

  void update()
  {
    mModelContextList.update(*this);
    // FIXME
//     mMechanicContextList.update(*this);
  }

  ModelContextList mModelContextList;
  MechanicContextList mMechanicContextList;

private:
  real_type mStepsize;
};

typedef std::list<SharedPtr<DiscreteTask> > DiscreteTaskList;

class TaskScheduler {
public:
  DiscreteTaskList mDiscreteTaskList;
  SharedPtr<ContinousTask> mContinousTask;
};

class System : public Object {
public:
  System(const std::string& name, Node* node = 0) :
    Object(name),
    mNode(node)
  { }

  SharedPtr<Node> getNode()
  { return mNode; }
  SharedPtr<const Node> getNode() const
  { return mNode; }
  void setNode(Node* node)
  {
    clear();
    mNode = node;
  }

  bool init()
  {
    if (!mNode)
      return true;

    // Build up the lists required to run the model.
    LeafInstanceCollector nodeInstanceCollector;
    mNode->accept(nodeInstanceCollector);
    
    // Allocates and distributes the PortValues, is required for the sort
    // steps below
    if (!nodeInstanceCollector.allocPortValues())
      return false;
    // The model instances are sorted to match the direct input property
    if (!nodeInstanceCollector.sortModelList())
      return false;

    ModelInstanceList modelContextList;
    nodeInstanceCollector.getModelContextList(modelContextList);
    // ...

    // Ok, all successful so far, get the lists from the visitor
    mNodeInstanceList.swap(nodeInstanceCollector._nodeInstanceList);

    return true;
  }

  void clear()
  {
    mNodeInstanceList.clear();
  }

  /// Simulate the system until the time tEnd
  bool simulate(real_type tEnd)
  {
    return false;
  }

  /// Bring the system in an equilibrum state near the current state ...
  bool trim(void)
  {
    return false;
  }

  /// Return the current simulation time, convenience function
//   const real_type& getTime(void) const
//   { return mTime; }


  const NodeInstanceList& getNodeInstanceList() const
  { return mNodeInstanceList; }

private:
  SharedPtr<Node> mNode;

  NodeInstanceList mNodeInstanceList;
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

  SharedPtr<System> system = new System("System", topGroup);

  if (!system->init())
    return 1;

  NodeInstanceList::const_iterator i;
  for (i = system->getNodeInstanceList().begin();
       i != system->getNodeInstanceList().end(); ++i) {
    std::cout << (*i)->getNodeNamePath() << std::endl;
    for (unsigned k = 0; k < (*i)->getNode().getNumPorts(); ++k) {
      std::cout << "  " << (*i)->getNode().getPort(k)->getName() << " "
                << (*i)->getPortValueList().getPortValue(k);
      const NumericPortValue* npv =
        dynamic_cast<const NumericPortValue*>((*i)->getPortValueList().getPortValue(k));
      if (npv)
        std::cout << " " << npv->getValue();
      std::cout << std::endl;
    }
  }

  return 0;
}

// Kabelbaum <-> PortBundle ??? Original Kabelbaum == Cabel Bundle
// Oder Cable Set <-> Port Set???

