/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#include "System.h"

#include "AbstractSystem.h"
#include "ConstNodeVisitor.h"
#include "Environment.h"
#include "Group.h"
#include "GroupInterfaceNode.h"
#include "Interact.h"
#include "Joint.h"
#include "NodeInstance.h"
#include "Object.h"
#include "RigidBody.h"
#include "RootJoint.h"
#include "SystemOutput.h"
#include "Task.h"
#include "TopologySort.h"
#include "TypeInfo.h"
#include "Variant.h"

#include "Function.h"
#include "DoPri5.h"

namespace OpenFDM {


/// System bootstrap:
///
/// The first step is to collect all the structural and leaf nodes in
/// the whole system. For each instance we have a data struct that
/// references such a model.
/// During that traversal, the port connections are assigned and connected
/// port values get a single data structure assigned, that is used to
/// distribute port values to the model nodes.
/// Models and mechanic nodes are sorted according to their direct input
/// dependencies and parent child relationship for mechanic nodes.
/// Then the port values itself are allocated.
/// Models are checkd for unset port values.
/// Past that, the contexts are allocated. This allocation is delegated to the
/// leaf model nodes so that they can alloacte contexts dependent of the port
/// values that are actually attached to the model.
/// Once this is done, the models are distributed to the actual execution lists
/// and AbstractNodeInstance's are allocated to reflect the ready to run
/// System to the user.
///
///
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

class ContinousSystemFunction : public Function {
public:
  virtual size_type inSize(void) const
  { return mContinousTask->getNumStates(); }
  virtual size_type outSize(void) const
  { return mContinousTask->getNumStates(); }
  virtual void eval(value_type t, const invector_type& v, outvector_type& out)
  {
    mContinousTask->setStateValue(v);
    mContinousTask->output(t);
    mContinousTask->derivative();
    mContinousTask->getStateDerivative(out);
  }
  SharedPtr<ContinousTask> mContinousTask;
};

class DiscreteSystem : public AbstractSystem {
  struct TimeSlice;
public:
  DiscreteSystem(const real_type& basicSampleTime, unsigned numSteps) :
    mCycleTime(basicSampleTime*numSteps),
    mBasicSampleTime(basicSampleTime)
  {
    DiscreteTask *discreteTask = new DiscreteTask(basicSampleTime);
    TimeSlice timeSlice;
    timeSlice.mTaskIndex.push_back(0);
    mDiscreteTaskList.push_back(discreteTask);
    mList.push_back(timeSlice);

    mContinousTask = new ContinousTask;
    mInitTask = new InitTask;

    mContinousSystemFunction = new ContinousSystemFunction;
    mContinousSystemFunction->mContinousTask = mContinousTask;

    mODESolver = new DoPri5;
    mODESolver->setFunction(mContinousSystemFunction);
  }

  void appendModelContext(const SampleTime& sampleTime,
                          AbstractModelContext* modelContext,
                          unsigned listIndex)
  {
    // The init task contains them all
    mInitTask->mModelContextList[listIndex].push_back(modelContext);
    
    // for now continous tasks take also all of them
    mContinousTask->appendStateValuesFromLeafContext(*modelContext);
    if (sampleTime.isContinous())
      mContinousTask->mModelContextList[listIndex].push_back(modelContext);

    // Discrete tasks need special treatment
    if (sampleTime.isDiscrete()) {
      real_type realSampleTime = sampleTime.getSampleTime().getRealValue();
      for (unsigned i = 0; i < mDiscreteTaskList.size(); ++i) {
        if (!equal(mDiscreteTaskList[i]->getStepsize(), realSampleTime))
          continue;
        // FIXME: is this true??? No model index here??
        mDiscreteTaskList[i]->mModelContextList.push_back(modelContext);
        break;
      }
    }
  }

  void appendMechanicContext(MechanicContext* mechanicContext)
  {
    // The init task contains them all
    mInitTask->mMechanicContextList.push_back(mechanicContext);
    
    // for now continous tasks take also all of them
    mContinousTask->appendStateValuesFromLeafContext(*mechanicContext);
    mContinousTask->mMechanicContextList.push_back(mechanicContext);
  }

protected:
  virtual void initImplementation(const real_type& t)
  {
    mInitTask->init(t);

    // Set the state into the ode solver
    Vector v;
    mContinousTask->getStateValue(v);
    mODESolver->setState(v);
    mODESolver->setTime(t);

    if (mList.empty())
      return;

    mList.front().mSampleHit = t;
  }

  virtual void discreteUpdateImplementation()
  {
    if (mList.empty())
      return;
    // std::rotate(...) ... nein - ist ineffizient ...
    discreteUpdate(mList.front());
    // FIXME: do something that does not creep away ...
    mList.front().mSampleHit += mCycleTime;
    discreteOutput(mList.front());

    // FIXME:
    real_type startTime = getValidityInterval().getEnd();
    real_type endTime = mBasicSampleTime + getValidityInterval().getEnd();
    // Note that this implicitly sets the value of the next sample hit to the
    // end of this validity interval.
    setValidityInterval(TimeInterval(startTime, endTime));
  }
  void discreteUpdate(const TimeSlice& timeSlice)
  {
    for (unsigned i = 0; i < timeSlice.mTaskIndex.size(); ++i) {
      unsigned index = timeSlice.mTaskIndex[i];
      mDiscreteTaskList[index]->update(timeSlice.mSampleHit);
    }
  }
  // FIXME ???
  void discreteOutput(const TimeSlice& timeSlice)
  {
    for (unsigned i = 0; i < timeSlice.mTaskIndex.size(); ++i) {
      unsigned index = timeSlice.mTaskIndex[i];
      mDiscreteTaskList[index]->output(timeSlice.mSampleHit);
    }
  }

  virtual void continousUpdateImplementation(const real_type& tEnd)
  {
    mODESolver->integrate(tEnd);
    mContinousTask->setStateValue(mODESolver->getState());
    output(mODESolver->getTime());
  }
  virtual void outputImplementation(const real_type& t)
  {
    if (mContinousTask)
      mContinousTask->output(t);
  }

private:
  struct TimeSlice {
    /// The list of task indices that must be executed at this sample hit
    std::vector<unsigned> mTaskIndex;
    /// The time of the next sample hit
    real_type mSampleHit;
    /// Helps to get less drifting time values:
    /// mCycleOffset*mCycleTime is the time offset of this slice in a complete
    /// cycle of all slices ...
    //const real_type mCycleOffset;
  };

  /// The time for all timeslices in the system.
  /// Used to increment the sample hit times ...
  /// FIXME
  const real_type mCycleTime;
  const real_type mBasicSampleTime;

  /// The Task responsible for initializing the model contexts.
  /// Contains all node contexts in execution order.
  /// All other tasks just contain their relevant subsets.
  SharedPtr<InitTask> mInitTask;
  /// All discrete Tasks in this System.
  std::vector<SharedPtr<DiscreteTask> > mDiscreteTaskList;
  /// The time slice sorted list of tasks to execute on each update.
  /// Past each update this list is rotated by one entry.
  std::list<TimeSlice> mList;
  /// The continous task that is used to compute continous outputs.
  /// (should! FIXME) Contains only those tasks that change on continous output.
  SharedPtr<ContinousTask> mContinousTask;
  SharedPtr<ContinousSystemFunction> mContinousSystemFunction;
  SharedPtr<ODESolver> mODESolver;
};

////////////////////////////////////////////////////////////////////

class System::NodeInstanceCollector : public ConstNodeVisitor {
public:
  NodeInstanceCollector(const SampleTime& sampleTime,
                        Environment* environment) :
    mSampleTime(sampleTime),
    mBasicSampleTime(sampleTime),
    mEnvironment(environment)
  { }

  struct PortData;

  struct PortConnectSet : public Referenced {
    bool setPortValue(PortValue* portValue)
    {
      if (!portValue)
        return false;
      mPortValue = portValue;
      return true;
    }
    PortValue* getPortValue()
    {
      return mPortValue;
    }
    std::vector<WeakPtr<PortData> > mParentPortData;
    SharedPtr<PortValue> mPortValue;
  };

  struct PortData : public WeakReferenced {
  public:
    PortData(const Port* port, bool valueCreator = true) :
      mPort(port),
      mNumConnectedPorts(0),
      mPortValueCreator(valueCreator)
    {
      getOrCreatePortConnectSet();
    }

    void setPortConnectSet(PortConnectSet* portSet)
    {
      OpenFDMAssert(portSet);
      if (portSet == mPortConnectSet)
        return;
      if (!mPortConnectSet) {
        mPortConnectSet = portSet;
        mPortConnectSet->mParentPortData.push_back(this);
        return;
      }
      // Merge the port sets together ...
      while (!mPortConnectSet->mParentPortData.empty()) {
        SharedPtr<PortData> portData;
        portData = mPortConnectSet->mParentPortData.back().lock();
        mPortConnectSet->mParentPortData.pop_back();
        if (portData == this)
          continue;
        portData->mPortConnectSet = portSet;
        portSet->mParentPortData.push_back(portData);
      }
      mPortConnectSet = portSet;
      mPortConnectSet->mParentPortData.push_back(this);
    }

    PortConnectSet* getOrCreatePortConnectSet()
    {
      if (mPortConnectSet)
        return mPortConnectSet;
      setPortConnectSet(new PortConnectSet);
      return mPortConnectSet;
    }

    bool addPortData(PortData* portData)
    {
      if (getPort()->getMaxConnects() <= mNumConnectedPorts)
        return false;
      mNumConnectedPorts += 1;
      setPortConnectSet(portData->getOrCreatePortConnectSet());
      return true;
    }

    bool isConnected(const PortData& portData) const
    {
      return mPortConnectSet && mPortConnectSet == portData.mPortConnectSet;
    }

    bool connect(PortData* portData)
    {
      if (getPort()->getMaxConnects() <= mNumConnectedPorts)
        return false;
      if (!portData->addPortData(this))
        return false;
      mNumConnectedPorts += 1;
      setPortConnectSet(portData->getOrCreatePortConnectSet());
      return true;
    }

    const SharedPtr<const Port>& getPort() const
    { return mPort; }

    void setProxyPortData(PortData* proxyPortData)
    {
      mPortValueCreator = false;
      setPortConnectSet(proxyPortData->getOrCreatePortConnectSet());
    }

    bool createPortValue()
    {
      if (!mPortValueCreator)
        return true;
      if (getPortValue())
        return true;
      SharedPtr<PortValue> portValue = getPort()->newValue();
      if (!portValue)
        return true; // FIXME
      if (!mPortConnectSet->setPortValue(portValue))
        return false;
      return true;
    }

    PortValue* getPortValue()
    {
      return mPortConnectSet->getPortValue();
    }

  private:
    SharedPtr<const Port> mPort;
    // The number of *directly* connected ports. That is those direct
    // connections within the current Group. Is used to ensure 1:* port connects
    // if required by the Port.
    unsigned mNumConnectedPorts;
    SharedPtr<PortConnectSet> mPortConnectSet;
    // Hmm, should this go into Port??
    bool mPortValueCreator;
  };

  struct InstanceData : public WeakReferenced {
    InstanceData(const Node& node, const NodePath& nodePath,
                 const SampleTime& sampleTime) :
      mNodePath(nodePath),
      mSampleTime(sampleTime)
    {
      unsigned numPorts = node.getNumPorts();
      mPortDataVector.reserve(numPorts);
      for (unsigned i = 0; i < numPorts; ++i)
        mPortDataVector.push_back(new PortData(node.getPort(i)));
    }
    virtual ~InstanceData()
    { }
    virtual const Node* getNode() const = 0;
    virtual AbstractNodeInstance* newNodeInstance() = 0;

    PortData* getPortData(unsigned i)
    {
      // Internal used function, just cry if this does not hold
      OpenFDMAssert(i < mPortDataVector.size());
      return mPortDataVector[i];
    }

    bool
    dependsOn(const InstanceData& instance, bool acceleration = false) const
    {
      unsigned numPorts = getNode()->getNumPorts();
      for (unsigned i = 0; i < numPorts; ++i) {
        const InputPort* inputPort;
        inputPort = getNode()->getPort(i)->toInputPort();
        if (!inputPort)
          continue;
        if (!inputPort->getDirectInput())
          continue;
        OpenFDMAssert(i < mPortDataVector.size());

        unsigned otherNumPorts = instance.getNode()->getNumPorts();
        for (unsigned j = 0; j < otherNumPorts; ++j) {
          const OutputPort* outputPort;
          outputPort = instance.getNode()->getPort(j)->toOutputPort();
          if (!outputPort)
            continue;
          if (!acceleration && outputPort->getAccelerationOutput())
            continue;

          OpenFDMAssert(j < instance.mPortDataVector.size());
          OpenFDMAssert(instance.mPortDataVector[j]);
          if (!mPortDataVector[i]->isConnected(*instance.mPortDataVector[j]))
            continue;
          
          return true;
        }
      }
      return false;
    }

    std::string getNodeNamePath() const
    { return Node::toNodePathName(mNodePath); }

    bool createPortValues()
    {
      for (unsigned i = 0; i < mPortDataVector.size(); ++i) {
        Log(Schedule, Debug3) << "Try to create port value \""
                              << mPortDataVector[i]->getPort()->getName()
                              << "\" of \"" << getNodeNamePath()
                              << "\"" << std::endl;
        if (!mPortDataVector[i]->createPortValue()) {
          Log(Schedule, Warning) << "Failed to create port value \""
                                 << mPortDataVector[i]->getPort()->getName()
                                 << "\" of \"" << getNodeNamePath()
                                 << "\".\nAborting!" << std::endl;

          return false;
        }
      }
      return true;
    }

    bool fetchPortValues()
    {
      for (unsigned i = 0; i < mPortDataVector.size(); ++i) {
        Log(Schedule, Debug3) << "Try to fetch port value \""
                              << mPortDataVector[i]->getPort()->getName()
                              << "\" of \"" << getNodeNamePath()
                              << "\"" << std::endl;
        PortValue* portValue = mPortDataVector[i]->getPortValue();
        if (!portValue) {
          Log(Schedule, Warning) << "Failed to fetch port value \""
                                 << mPortDataVector[i]->getPort()->getName()
                                 << "\" of \"" << getNodeNamePath()
                                 << "\".\nAborting!" << std::endl;
          return false;
        }
        if (!mPortDataVector[i]->getPort()->acceptPortValue(portValue)) {
          Log(Schedule, Warning) << "Failed to accept fetched port value \""
                                 << mPortDataVector[i]->getPort()->getName()
                                 << "\" of \"" << getNodeNamePath()
                                 << "\".\nAborting!" << std::endl;
          return false;
        }

        mPortValueList.setPortValue(i, portValue);
      }
      return true;
    }

    const SampleTime& getSampleTime() const
    { return mSampleTime; }

  protected:
    const NodePath mNodePath;
    const SampleTime mSampleTime;

    typedef std::vector<SharedPtr<PortData> > PortDataVector;
    PortDataVector mPortDataVector;

    PortValueList mPortValueList;
  };

  struct NodeInstanceData : public InstanceData {
    NodeInstanceData(const Node& node, const NodePath& nodePath,
                     const SampleTime& sampleTime) :
      InstanceData(node, nodePath, sampleTime),
      mNode(&node)
    { }
    virtual const Node* getNode() const { return mNode; }
    virtual AbstractNodeInstance* newNodeInstance()
    { return new NodeInstance(mSampleTime, getNode(), mPortValueList); }
  private:
    SharedPtr<const Node> mNode;
  };
  struct ModelInstanceData : public InstanceData {
    ModelInstanceData(const AbstractModel& model, const NodePath& nodePath,
                      const SampleTime& sampleTime) :
      InstanceData(model, nodePath, sampleTime),
      mModel(&model)
    { }
    virtual const AbstractModel* getNode() const { return mModel; }

    bool createModelContext()
    {
      OpenFDMAssert(!mModelContext);
      mModelContext = mModel->newModelContext(mPortValueList);
      if (!mModelContext) {
        Log(Schedule, Warning) << "Could not create context for model \""
                               << getNodeNamePath() << "\"" << std::endl;
        return false;
      }
      return true;
    }

    virtual AbstractNodeInstance* newNodeInstance()
    { return new LeafInstance(mSampleTime, mModelContext); }

    AbstractModelContext* getModelContext()
    { return mModelContext; }

  private:
    SharedPtr<const AbstractModel> mModel;
    SharedPtr<AbstractModelContext> mModelContext;
  };
  struct MechanicInstanceData : public InstanceData {
    MechanicInstanceData(const MechanicNode& mechanicNode,
                         const NodePath& nodePath,
                         const SampleTime& sampleTime) :
      InstanceData(mechanicNode, nodePath, sampleTime)
    { }
    virtual const MechanicNode* getNode() const = 0;

    virtual bool createMechanicContext(Environment*) = 0;

    virtual AbstractNodeInstance* newNodeInstance()
    { return new LeafInstance(mSampleTime, mMechanicContext); }

    MechanicContext* getMechanicContext()
    { return mMechanicContext; }

    // Currently unused
    bool isLinkedTo(const MechanicInstanceData& instance) const
    {
      unsigned numPorts = getNode()->getNumPorts();
      for (unsigned i = 0; i < numPorts; ++i) {
        if (!getNode()->getPort(i)->toMechanicLink())
          continue;
        OpenFDMAssert(i < mPortDataVector.size());

        const Node* otherNode = instance.getNode();
        unsigned otherNumPorts = otherNode->getNumPorts();
        for (unsigned j = 0; j < otherNumPorts; ++j) {
          if (!otherNode->getPort(j)->toMechanicLink())
            continue;

          OpenFDMAssert(j < instance.mPortDataVector.size());
          OpenFDMAssert(instance.mPortDataVector[j]);
          if (!mPortDataVector[i]->isConnected(*instance.mPortDataVector[j]))
            continue;
          
          return true;
        }
      }
      return false;
    }
  protected:
    SharedPtr<MechanicContext> mMechanicContext;
  };
  struct JointInstanceData : public MechanicInstanceData {
    JointInstanceData(const Joint& joint, const NodePath& nodePath,
                      const SampleTime& sampleTime) :
      MechanicInstanceData(joint, nodePath, sampleTime),
      mJoint(&joint)
    { }
    virtual const Joint* getNode() const { return mJoint; }
    virtual bool createMechanicContext(Environment* environment)
    {
      OpenFDMAssert(!mMechanicContext);
      mMechanicContext = getNode()->newMechanicContext(environment, mParentLink,
                                                       mChildLink,
                                                       mPortValueList);
      if (!mMechanicContext) {
        Log(Schedule, Warning) << "Could not create context for mechanic "
                               << "node \"" << getNodeNamePath()
                               << "\"" << std::endl;
        return false;
      }
      return true;
    }

    bool isLinkedToAndSetChild(JointInstanceData& instance)
    {
      unsigned numPorts = getNode()->getNumPorts();
      for (unsigned i = 0; i < numPorts; ++i) {
        const MechanicLink* mechanicLink;
        mechanicLink = getNode()->getPort(i)->toMechanicLink();
        if (!mechanicLink)
          continue;
        OpenFDMAssert(i < mPortDataVector.size());

        const Node* otherNode = instance.getNode();
        unsigned otherNumPorts = otherNode->getNumPorts();
        for (unsigned j = 0; j < otherNumPorts; ++j) {
          const MechanicLink* otherMechanicLink;
          otherMechanicLink = otherNode->getPort(j)->toMechanicLink();
          if (!otherMechanicLink)
            continue;

          OpenFDMAssert(j < instance.mPortDataVector.size());
          OpenFDMAssert(instance.mPortDataVector[j]);
          if (!mPortDataVector[i]->isConnected(*instance.mPortDataVector[j]))
            continue;

          mChildLink = mechanicLink;
          instance.mParentLink = otherMechanicLink;
          return true;
        }
      }
      return false;
    }

    // detect if this child link is connected to any mechanic port of instance
    bool isChildLinkedTo(const JointInstanceData& instance) const
    {
      unsigned i = mChildLink->getIndex();
      OpenFDMAssert(i < mPortDataVector.size());
      OpenFDMAssert(mPortDataVector[i]);

      const Node* otherNode = instance.getNode();
      unsigned otherNumPorts = otherNode->getNumPorts();
      for (unsigned j = 0; j < otherNumPorts; ++j) {
        if (!otherNode->getPort(j)->toMechanicLink())
          continue;
        
        OpenFDMAssert(j < instance.mPortDataVector.size());
        OpenFDMAssert(instance.mPortDataVector[j]);
        if (!mPortDataVector[i]->isConnected(*instance.mPortDataVector[j]))
          continue;
        
        return true;
      }
      return false;
    }

    // FIXME: do real checks in this case, connected and so forth ...
    bool makeRemainigLinksChildLinks()
    {
      unsigned numPorts = getNode()->getNumPorts();
      for (unsigned i = 0; i < numPorts; ++i) {
        const MechanicLink* mechanicLink;
        mechanicLink = getNode()->getPort(i)->toMechanicLink();
        if (!mechanicLink)
          continue;
        if (mechanicLink == mParentLink)
          continue;
        mChildLink = mechanicLink;
      }
      return true;
    }

    SharedPtr<const MechanicLink> mParentLink;
    SharedPtr<const MechanicLink> mChildLink;
  private:
    SharedPtr<const Joint> mJoint;
  };
  struct InteractInstanceData : public MechanicInstanceData {
    InteractInstanceData(const Interact& interact, const NodePath& nodePath,
                         const SampleTime& sampleTime) :
      MechanicInstanceData(interact, nodePath, sampleTime),
      mInteract(&interact)
    { }
    virtual const Interact* getNode() const { return mInteract; }
    virtual bool createMechanicContext(Environment* environment)
    {
      OpenFDMAssert(!mMechanicContext);
      mMechanicContext = getNode()->newMechanicContext(environment,
                                                       mPortValueList);
      if (!mMechanicContext) {
        Log(Schedule, Warning) << "Could not create context for mechanic "
                               << "node \"" << getNodeNamePath()
                               << "\"" << std::endl;
        return false;
      }
      return true;
    }
  private:
    SharedPtr<const Interact> mInteract;
  };

  void addInstanceData(InstanceData* instanceData)
  {
    // Add the instance to the per System instance map
    mInstanceDataMap[getNodePath()] = instanceData;
    // Add the instance to the current groups instance list
    mInstanceDataVector.push_back(instanceData);
  }

  virtual void apply(const Node& node)
  { Log(Schedule, Error) << __PRETTY_FUNCTION__ << std::endl; }
  virtual void apply(const MechanicNode& node)
  { Log(Schedule, Error) << __PRETTY_FUNCTION__ << std::endl; }
  virtual void apply(const LeafNode& leaf)
  { Log(Schedule, Error) << __PRETTY_FUNCTION__ << std::endl; }
  virtual void apply(const LibraryNode& libraryNode)
  { Log(Schedule, Error) << __PRETTY_FUNCTION__ << std::endl; }

  virtual void apply(const GroupInterfaceNode& node)
  {
    SampleTime sampleTime = mergeSampleTime(mSampleTime, node.getSampleTime());
    SharedPtr<NodeInstanceData> instanceData;
    instanceData = new NodeInstanceData(node, getNodePath(), sampleTime);
    addInstanceData(instanceData);
    mNodeInstanceDataList.push_back(instanceData);

    OpenFDMAssert(node.getPort(0));
    PortData* portData = instanceData->getPortData(0);
    OpenFDMAssert(portData);

    PortData* parentPortData;
    parentPortData = mGroupInterfacePortDataMap[node.getExternalPortIndex()];
    if (!parentPortData) {
      Log(Schedule, Error) << "Internal Error: Cannot find external port "
        "data for group external port!" << std::endl;
      return;
    }

    parentPortData->setProxyPortData(portData);
    portData->setProxyPortData(parentPortData);
  }

  virtual void apply(const RootJoint& node)
  {
    // Need to store the root nodes to build up the spanning tree for the
    // mechanical system here.
    SampleTime sampleTime = mergeSampleTime(mSampleTime, node.getSampleTime());
    SharedPtr<JointInstanceData> instanceData;
    instanceData = new JointInstanceData(node, getNodePath(), sampleTime);
    addInstanceData(instanceData);
    mRootJointInstanceDataList.push_back(instanceData);
  }
  virtual void apply(const Interact& node)
  {
    SampleTime sampleTime = mergeSampleTime(mSampleTime, node.getSampleTime());
    SharedPtr<InteractInstanceData> instanceData;
    instanceData = new InteractInstanceData(node, getNodePath(), sampleTime);
    addInstanceData(instanceData);
    mInteractInstanceDataList.push_back(instanceData);
  }
  virtual void apply(const RigidBody& node)
  {
    SampleTime sampleTime = mergeSampleTime(mSampleTime, node.getSampleTime());
    SharedPtr<NodeInstanceData> instanceData;
    instanceData = new NodeInstanceData(node, getNodePath(), sampleTime);
    addInstanceData(instanceData);
    mNodeInstanceDataList.push_back(instanceData);

    // Make all rigid mechanic body links use the same link value
    // FIXME, allocate them in this way!
    PortData* portData = 0;
    for (unsigned i = 0; i < node.getNumPorts(); ++i) {
      if (!node.getPort(i)->toMechanicLink())
        continue;
      if (portData) {
        instanceData->getPortData(i)->setProxyPortData(portData);
      } else {
        portData = instanceData->getPortData(i);
      }
    }
  }
  virtual void apply(const Joint& node)
  {
    SampleTime sampleTime = mergeSampleTime(mSampleTime, node.getSampleTime());
    SharedPtr<JointInstanceData> instanceData;
    instanceData = new JointInstanceData(node, getNodePath(), sampleTime);
    addInstanceData(instanceData);
    mJointInstanceDataList.push_back(instanceData);
  }
  virtual void apply(const AbstractModel& node)
  {
    SampleTime sampleTime = mergeSampleTime(mSampleTime, node.getSampleTime());
    SharedPtr<ModelInstanceData> instanceData;
    instanceData = new ModelInstanceData(node, getNodePath(), sampleTime);
    addInstanceData(instanceData);
    mModelInstanceDataList.push_back(instanceData);
  }

  virtual void apply(const Group& group)
  {
    // push the sample time
    SampleTime parentSampleTime = mSampleTime;
    mSampleTime = mergeSampleTime(parentSampleTime, group.getSampleTime());

    SharedPtr<NodeInstanceData> instanceData;
    instanceData = new NodeInstanceData(group, getNodePath(), mSampleTime);
    addInstanceData(instanceData);
    mNodeInstanceDataList.push_back(instanceData);

    // The vector of instances for this group.
    InstanceDataVector parentInstanceDataVector;
    parentInstanceDataVector.swap(mInstanceDataVector);
    mInstanceDataVector.reserve(group.getNumChildren());

    // Get PortDataList indexed by group port index
    GroupInterfacePortDataMap parentGroupInterfacePortDataMap;
    mGroupInterfacePortDataMap.swap(parentGroupInterfacePortDataMap);
    mGroupInterfacePortDataMap.resize(group.getNumPorts());
    // Alloc PortData structs of each group port
    for (unsigned i = 0; i < mGroupInterfacePortDataMap.size(); ++i)
      mGroupInterfacePortDataMap[i] = instanceData->getPortData(i);

    // End pushing external connection data

    // Now walk the children
    for (unsigned i = 0; i < group.getNumChildren(); ++i) {

      // our next node to traverse
      SharedPtr<const Node> node = group.getChild(i);

      // now traverse the child ...
      node->accept(*this);
    }

    // Apply the group internal connections to the instances
    unsigned numConnects = group.getNumConnects();
    for (unsigned i = 0; i < numConnects; ++i) {
      SharedPtr<const Connect> connect = group.getConnect(i);
      if (!connect) {
        Log(Schedule, Warning) << "Zero Connect pointer #" << i
                               << " for Group \""
                               << Node::toNodePathName(getNodePath())
                               << "\"" << std::endl;
        continue;
      }

      SharedPtr<const Port> port0 = connect->getPort0();
      if (!port0) {
        Log(Schedule, Warning) << "Incomplete connect #" << i << " for Group \""
                               << Node::toNodePathName(getNodePath())
                               << "\"" << std::endl;
        continue;
      }
      SharedPtr<const Port> port1 = connect->getPort1();
      if (!port1) {
        Log(Schedule, Warning) << "Incomplete connect #" << i << " for Group \""
                               << Node::toNodePathName(getNodePath())
                               << "\"" << std::endl;
        continue;
      }

      SharedPtr<const Node> node0 = port0->getNode();
      if (!node0) {
        Log(Schedule, Warning) << "Dangling port #0 connected in connect #" << i
                               << " for Group \""
                               << Node::toNodePathName(getNodePath())
                               << "\"" << std::endl;
        continue;
      }
      SharedPtr<const Node> node1 = port1->getNode();
      if (!node1) {
        Log(Schedule, Warning) << "Dangling port #1 connected in connect #" << i
                               << " for Group \""
                               << Node::toNodePathName(getNodePath())
                               << "\"" << std::endl;
        continue;
      }
      unsigned nodeIndex0 = group.getChildIndex(node0);
      if (nodeIndex0 == ~0u) {
        Log(Schedule, Warning) << "Node #0 does no longer belong to group \""
                               << Node::toNodePathName(getNodePath())
                               << "\" in connect #" << i << std::endl;
        continue;
      }
      unsigned nodeIndex1 = group.getChildIndex(node1);
      if (nodeIndex1 == ~0u) {
        Log(Schedule, Warning) << "Node #1 does no longer belong to group \""
                               << Node::toNodePathName(getNodePath())
                               << "\" in connect #" << i << std::endl;
        continue;
      }

      unsigned portIndex0 = port0->getIndex();
      unsigned portIndex1 = port1->getIndex();
      if (!mInstanceDataVector[nodeIndex1]->getPortData(portIndex1)->
          connect(mInstanceDataVector[nodeIndex0]->getPortData(portIndex0)))
        Log(Schedule, Error) << "Internal Error: Cannot connect ports that"
          " appeared to be compatible before." << std::endl;
    }

    // We must have gained exactly this amount of instances while traversing
    // this group, so make sure it is like that ...
    OpenFDMAssert(mInstanceDataVector.size() == group.getNumChildren());

    // Pop the per group port connect info
    parentGroupInterfacePortDataMap.swap(mGroupInterfacePortDataMap);
    parentInstanceDataVector.swap(mInstanceDataVector);

    // restore old group sample time
    mSampleTime = parentSampleTime;
  }

  ////////////////////////////////////////////////////////////////////////////
  // All instances in the system indexed by node path.
  typedef std::map<NodePath, SharedPtr<InstanceData> > InstanceDataMap;
  InstanceDataMap mInstanceDataMap;

  typedef std::list<SharedPtr<NodeInstanceData> > NodeInstanceDataList;
  typedef std::list<SharedPtr<ModelInstanceData> > ModelInstanceDataList;
  typedef std::list<SharedPtr<MechanicInstanceData> > MechanicInstanceDataList;
  typedef std::list<SharedPtr<JointInstanceData> > JointInstanceDataList;

  // The list of Nodes that do not need a context for itself.
  NodeInstanceDataList mNodeInstanceDataList;
  // The Models list, worthwhile for sorting
  ModelInstanceDataList mModelInstanceDataList;
  ModelInstanceDataList mVelocityModelInstanceDataList;
  ModelInstanceDataList mAccelerationModelInstanceDataList;
  // The mechanical system list, also for sorting
  MechanicInstanceDataList mMechanicInstanceDataList;

  // The list of root nodes in the mechanical system. Will be a starting point
  // for sorting the tree of mechanical models downwards
  JointInstanceDataList mRootJointInstanceDataList;
  JointInstanceDataList mJointInstanceDataList;
  MechanicInstanceDataList mInteractInstanceDataList;

  ////////////////////////////////////////////////////////////////////////////
  // Used to map connections in groups ...
  typedef std::vector<SharedPtr<InstanceData> > InstanceDataVector;
  InstanceDataVector mInstanceDataVector;
  // Holds the PortDataList pointer indexed by parent groups port index
  typedef std::vector<SharedPtr<PortData> > GroupInterfacePortDataMap;
  GroupInterfacePortDataMap mGroupInterfacePortDataMap;

  // Current nodes sample time
  SampleTime mSampleTime;
  // past all the traversal, this contains the basic sample time of the
  // whole system. It is built up during traversal and has almost no meaning
  // until all models have be traversed.
  SampleTime mBasicSampleTime;

  // The mechanic simulation environment for this system
  SharedPtr<Environment> mEnvironment;

  // Here the miracle occurs.
  // The collected simulation nodes are packed into something that can be used
  // to simulate the system.
  AbstractSystem* buildSystem()
  {
    // Build the spanning tree for the mechanic system
    if (!sortMechanicList())
      return 0;
    // The model instances are sorted to match the direct input property
    if (!sortModelList())
      return 0;
    // Allocates and distributes the PortValues, check for unassigned ports
    // and allocate contexts.
    if (!createContexts())
      return 0;

    real_type basicSampleTime = 0.01; // FIXME in this case just continous
    if (mBasicSampleTime.isDiscrete())
      basicSampleTime = mBasicSampleTime.getSampleTime().getRealValue();

    Log(Model, Info) << "Basic sample time is " << basicSampleTime << std::endl;

    // Now the system is ready for state initialization and execution
    // Build up te abstract system and return that

    // For the first cut, assume many things like basic step size and such ...
    SharedPtr<DiscreteSystem> discreteSystem;
    discreteSystem = new DiscreteSystem(basicSampleTime, 1);

    ModelInstanceDataList::const_iterator i;
    for (i = mModelInstanceDataList.begin();
         i != mModelInstanceDataList.end(); ++i) {
      discreteSystem->appendModelContext((*i)->getSampleTime(),
                                         (*i)->getModelContext(), 0);
    }

    for (i = mVelocityModelInstanceDataList.begin();
         i != mVelocityModelInstanceDataList.end(); ++i) {
      discreteSystem->appendModelContext((*i)->getSampleTime(),
                                         (*i)->getModelContext(), 1);
    }

    for (i = mAccelerationModelInstanceDataList.begin();
         i != mAccelerationModelInstanceDataList.end(); ++i) {
      discreteSystem->appendModelContext((*i)->getSampleTime(),
                                         (*i)->getModelContext(), 2);
    }

    MechanicInstanceDataList::const_iterator j;
    for (j = mMechanicInstanceDataList.begin();
         j != mMechanicInstanceDataList.end(); ++j) {
      discreteSystem->appendMechanicContext((*j)->getMechanicContext());
    }

    return discreteSystem.release();
  }

protected:


  SampleTime mergeSampleTime(const SampleTime& parentSampleTime,
                             const SampleTime& nodeSampleTime)
  {
    if (nodeSampleTime.isInherited())
      return parentSampleTime;

    SampleTime sampleTime = nodeSampleTime;
    if (sampleTime.isDiscrete()) {
      if (!mBasicSampleTime.isDiscrete())
        mBasicSampleTime = sampleTime;
      else {
        Fraction a = mBasicSampleTime.getSampleTime();
        Fraction b = sampleTime.getSampleTime();
        mBasicSampleTime = SampleTime(greatestCommonDivisor(a, b));
        OpenFDMAssert(mBasicSampleTime.isDiscrete());
      }
    }
    return sampleTime;
  }

  // method to sort the leafs according to their dependency
  bool sortMechanicList()
  {
    if (mRootJointInstanceDataList.empty() &&
        (!mJointInstanceDataList.empty() ||
         !mInteractInstanceDataList.empty())) {
      Log(Schedule,Warning)
        << "No root joint in System with mechanic components" << std::endl;
      return false;
    }

    // Start with all the roots in front of the list ...
    // FIXME: ensure that we have no root joints with two mechanic links.
    // They must then appear in the joint list. When we know that, we are sure
    // that there is no mechanic link loop in the root joints - they
    // have only one link ...
    JointInstanceDataList sortedJoints;
    sortedJoints.swap(mRootJointInstanceDataList);
    // For the case that the root joint is the only joint in the chain
    for (JointInstanceDataList::iterator i = sortedJoints.begin();
         i != sortedJoints.end(); ++i) {
      // FIXME
      (*i)->makeRemainigLinksChildLinks();
    }

    // Not the best algorithm, but for a first cut ...
    while (!mJointInstanceDataList.empty()) {
      JointInstanceDataList nextLevelList;

      JointInstanceDataList::iterator j;
      for (j = sortedJoints.begin();
           j != sortedJoints.end(); ++j) {
        JointInstanceDataList::iterator i;
        for (i = mJointInstanceDataList.begin();
             i != mJointInstanceDataList.end();) {
        
          if ((*j)->isLinkedToAndSetChild(*(*i))) {
            SharedPtr<JointInstanceData> mechanicInstanceData = *i;
            mechanicInstanceData->makeRemainigLinksChildLinks();
            nextLevelList.push_back(mechanicInstanceData);
            i = mJointInstanceDataList.erase(i);

            // Check if this current mechanic node does not reference
            // back into the already sorted models
            JointInstanceDataList::const_iterator k;
            for (k = sortedJoints.begin();
                 k != sortedJoints.end(); ++k) {
              if (*k == *j)
                continue;
              if (mechanicInstanceData->isChildLinkedTo(*(*k))) {
                Log(Schedule,Warning)
                  << "Detected closed kinematic loop: MechanicNode \""
                  << mechanicInstanceData->getNodeNamePath()
                  << "\" is linked to MechanicNode \""
                  << (*k)->getNodeNamePath() << "\"" << std::endl;
                return false;
              }
            }
          } else {
            ++i;
          }
        }
      }

      // Check if we have connects in this next level.
      // Since every mechanic node in this list already has a parent,
      // if we have a connection in between them, there must be a
      // closed kinematic loop.
      for (j = nextLevelList.begin(); j != nextLevelList.end(); ++j) {
        JointInstanceDataList::iterator i = j;
        for (++i; i != nextLevelList.end(); ++i) {
          if ((*j)->isChildLinkedTo(*(*i))) {
            Log(Schedule,Warning)
              << "Detected closed kinematic loop: MechanicNode \""
              << (*j)->getNodeNamePath()
              << "\" is linked to MechanicNode \""
              << (*i)->getNodeNamePath() << "\"" << std::endl;
            return false;
          }
        }
      }
      

      for (j = nextLevelList.begin(); j != nextLevelList.end(); ++j) {
        sortedJoints.push_back(*j);
      }
    }

    // First the sorted joints ...
    mMechanicInstanceDataList.assign(sortedJoints.begin(), sortedJoints.end());
    // ... then Interacts at the end of the list
    mMechanicInstanceDataList.splice(mMechanicInstanceDataList.end(),
                                     mInteractInstanceDataList,
                                     mInteractInstanceDataList.begin(),
                                     mInteractInstanceDataList.end());
    
    Log(Schedule,Info) << "MechanicNode Schedule" << std::endl;
    MechanicInstanceDataList::iterator i = mMechanicInstanceDataList.begin();
    for (; i != mMechanicInstanceDataList.end(); ++i) {
      Log(Schedule,Info)
        << "  MechanicNode \"" << (*i)->getNodeNamePath() << "\"" << std::endl;
    }

    return true;
  }

  bool isDirectVelocityDependent(ModelInstanceData& modelInstanceData)
  {
    MechanicInstanceDataList::iterator i = mMechanicInstanceDataList.begin();
    for (; i != mMechanicInstanceDataList.end(); ++i) {
      if (modelInstanceData.dependsOn(*(*i), false))
        return true;
      if (modelInstanceData.dependsOn(*(*i), true))
        return true;
    }
    return false;
  }

  bool isDirectForceModel(ModelInstanceData& modelInstanceData)
  {
    MechanicInstanceDataList::iterator i = mMechanicInstanceDataList.begin();
    for (; i != mMechanicInstanceDataList.end(); ++i) {
      if ((*i)->dependsOn(modelInstanceData))
        return true;
    }
    return false;
  }

  void
  getVelocityDependentModels(ModelInstanceDataList& velocityDependentModels)
  {
    // find all models that are dependent on the mechanic systems velocities
    ModelInstanceDataList::iterator i = mModelInstanceDataList.begin();
    while (i != mModelInstanceDataList.end()) {
      if (isDirectVelocityDependent(*(*i))) {
        velocityDependentModels.push_back(*i);
        i = mModelInstanceDataList.erase(i);
      } else {
        ++i;
      }
    }

    for (i = velocityDependentModels.begin();
         i != velocityDependentModels.end(); ++i) {

      ModelInstanceDataList::iterator j = mModelInstanceDataList.begin();
      while (j != mModelInstanceDataList.end()) {
        if ((*j)->dependsOn(*(*i))) {
          velocityDependentModels.push_back(*j);
          j = mModelInstanceDataList.erase(j);
        } else {
          ++j;
        }
      }
    }
  }

  void splitForceModels(ModelInstanceDataList& velocityDependentModels,
                        ModelInstanceDataList& forceModelList)
  {
    ModelInstanceDataList::iterator i = velocityDependentModels.begin();
    while (i != velocityDependentModels.end()) {
      if (isDirectForceModel(*(*i))) {
        forceModelList.push_back(*i);
        i = velocityDependentModels.erase(i);
      } else {
        ++i;
      }
    }

    for (i = forceModelList.begin();
         i != forceModelList.end(); ++i) {

      ModelInstanceDataList::iterator j = velocityDependentModels.begin();
      while (j != velocityDependentModels.end()) {
        if ((*i)->dependsOn(*(*j))) {
          forceModelList.push_back(*j);
          j = velocityDependentModels.erase(j);
        } else {
          ++j;
        }
      }
    }
  }

  bool sortModelList()
  {
    // How to sort:
    //  * First split out all velocity dependent models.
    //  * build up a usual sorted list of models
    //  * Go from the back of this list and push all models in the acceleration
    //    list when it is not needed for the forces (?? does not work ??)
    //    (hmm, alternatively build the force list by walking back the velocity
    //     list and pulling out all chains that depend on force.
    //     The old velocity list is than the acceleration one??)
    //  * Check the resulting velocity list for dependency on acceleration
    
    // get the list of models that depend on the mechnic systems velocity or
    // acceleration
    getVelocityDependentModels(mAccelerationModelInstanceDataList);

    // Seperate out those models that are really required for forces
    splitForceModels(mAccelerationModelInstanceDataList,
                     mVelocityModelInstanceDataList);

    // Sort that lists according to the usual rules
    if (!sortModelList(mModelInstanceDataList))
      return false;

    if (!sortModelList(mVelocityModelInstanceDataList))
      return false;

    if (!sortModelList(mAccelerationModelInstanceDataList))
      return false;

    return true;
  }

  struct ModelInstanceDataDepend {
    bool operator()(const SharedPtr<ModelInstanceData>& m0,
                    const SharedPtr<ModelInstanceData>& m1) const
    { return m0->dependsOn(*m1); }
  };

  static bool sortModelList(ModelInstanceDataList& modelInstanceDataList)
  {
    ModelInstanceDataList sortedModelInstanceDataList;
    tsort(sortedModelInstanceDataList, modelInstanceDataList,
          ModelInstanceDataDepend());

    // Check if we have some unsorted cycles left
    if (!modelInstanceDataList.empty()) {
      Log(Schedule,Warning) << "Detected cyclic loop:" << std::endl;

      ModelInstanceDataList::iterator i = modelInstanceDataList.begin();
      for (; i != modelInstanceDataList.end(); ++i) {
        Log(Schedule,Warning)
          << "  Model \"" << (*i)->getNodeNamePath() << "\"" << std::endl;
      }
      modelInstanceDataList.splice(modelInstanceDataList.begin(),
                                   sortedModelInstanceDataList);
      return false;
    }

    modelInstanceDataList.swap(sortedModelInstanceDataList);

    Log(Schedule,Info) << "Model Schedule" << std::endl;
    ModelInstanceDataList::iterator i = modelInstanceDataList.begin();
    for (; i != modelInstanceDataList.end(); ++i) {
      Log(Schedule,Info)
        << "  Model \"" << (*i)->getNodeNamePath() << "\"" << std::endl;
    }

    return true;
  }

  bool
  createContexts()
  {
    // alloc port values
    InstanceDataMap::const_iterator i;
    for (i = mInstanceDataMap.begin(); i != mInstanceDataMap.end(); ++i) {
      if (!i->second->createPortValues())
        return false;
    }

    // check port values and report unconnected values.
    for (i = mInstanceDataMap.begin(); i != mInstanceDataMap.end(); ++i) {
      if (!i->second->fetchPortValues())
        return false;
    }

    // Create the contexts
    // This happens past the port values are assigned, this way models can
    // create different kind of contexts based on the type of port values.
    ModelInstanceDataList::const_iterator j;
    for (j = mModelInstanceDataList.begin();
         j != mModelInstanceDataList.end(); ++j) {
      if (!(*j)->createModelContext())
        return false;
    }
    for (j = mVelocityModelInstanceDataList.begin();
         j != mVelocityModelInstanceDataList.end(); ++j) {
      if (!(*j)->createModelContext())
        return false;
    }
    for (j = mAccelerationModelInstanceDataList.begin();
         j != mAccelerationModelInstanceDataList.end(); ++j) {
      if (!(*j)->createModelContext())
        return false;
    }

    MechanicInstanceDataList::const_iterator k;
    for (k = mMechanicInstanceDataList.begin();
         k != mMechanicInstanceDataList.end(); ++k) {
      if (!(*k)->createMechanicContext(mEnvironment))
        return false;
    }

    return true;
  }
};

BEGIN_OPENFDM_OBJECT_DEF(System, Object)
  END_OPENFDM_OBJECT_DEF

System::System(const std::string& name, Node* node) :
  Object(name),
  mNode(node),
  mSampleTime(SampleTime::getContinous()),
  mEnvironment(new Environment)
{
}

System::~System()
{
}

void
System::setNode(Node* node)
{
  clear();
  mNode = node;
}

bool
System::setSampleTime(const SampleTime& sampleTime)
{
  if (!sampleTime.isContinous() && !sampleTime.isDiscrete()) {
    Log(Model, Warning) << "Ignoring attemp to set invalid sample time for "
                        << " System \"" << getName() << "\"" << std::endl;
    return false;
  }
  mSampleTime = sampleTime;
}

void
System::setEnvironment(Environment* environment)
{
  mEnvironment = environment;
}

Environment*
System::getEnvironment()
{
  return mEnvironment;
}

const Environment*
System::getEnvironment() const
{
  return mEnvironment;
}

bool
System::init(const real_type& t0)
{
  if (!mNode)
    return false;
  
  // Build up the lists required to run the model.
  NodeInstanceCollector nodeInstanceCollector(mSampleTime, mEnvironment);
  mNode->accept(nodeInstanceCollector);
  
  mAbstractSystem = nodeInstanceCollector.buildSystem();
  if (!mAbstractSystem)
    return false;

  // Have something to run in our hands.
  // Now get the information required to reflect the system to the user.
  NodeInstanceCollector::InstanceDataMap::const_iterator i;
  for (i = nodeInstanceCollector.mInstanceDataMap.begin();
       i != nodeInstanceCollector.mInstanceDataMap.end(); ++i) {
    mNodeInstanceMap[i->first] = i->second->newNodeInstance();
  }

  SystemOutputList::const_iterator j;
  for (j = mSystemOutputList.begin(); j != mSystemOutputList.end(); ++j)
    (*j)->setSystem(this);
  
  // Hmm, really here???
  mAbstractSystem->init(t0);

  return true;
}

void
System::clear()
{
  mAbstractSystem = 0;
  mNodeInstanceMap.clear();

  SystemOutputList::const_iterator i;
  for (i = mSystemOutputList.begin(); i != mSystemOutputList.end(); ++i)
    (*i)->setSystem(0);
}

/// Simulate the system until the time tEnd
bool
System::simulate(const real_type& t)
{
  if (!mAbstractSystem)
    return false;

  while (mAbstractSystem->getTime() < t) {
    mAbstractSystem->advance(t);

    SystemOutputList::const_iterator i;
    for (i = mSystemOutputList.begin(); i != mSystemOutputList.end(); ++i)
      (*i)->output(mAbstractSystem->getTime());
  }

  return true;
}

/// Bring the system in an equilibrum state near the current state ...
bool
System::trim(void)
{
  return false;
}

/// Return the current simulation time, convenience function
real_type
System::getTime(void) const
{
  if (!mAbstractSystem)
    return Limits<real_type>::quiet_NaN();
  return mAbstractSystem->getTime();
}

const AbstractNodeInstance*
System::getNodeInstance(const NodePath& nodePath) const
{
  NodeInstanceMap::const_iterator i = mNodeInstanceMap.find(nodePath);
  if (i == mNodeInstanceMap.end())
    return 0;
  return i->second;
}

void
System::attach(SystemOutput* systemOutput)
{
  if (!systemOutput)
    return;
  mSystemOutputList.push_back(systemOutput);
  if (!mAbstractSystem)
    return;
  systemOutput->setSystem(this);
}

void
System::detach(SystemOutput* systemOutput)
{
  if (!systemOutput)
    return;
  SystemOutputList::iterator i = mSystemOutputList.begin();
  while (i != mSystemOutputList.end()) {
    if (*i == systemOutput) {
      i = mSystemOutputList.erase(i);
      if (mAbstractSystem)
        systemOutput->setSystem(0);
    } else
      ++i;
  }
}

class System::NodeFinder : public NodeVisitor {
public:
  NodeFinder(const std::string& path) : mPath(path), mNode(0)
  { }
  virtual void apply(Group& group)
  {
    std::string path = Node::toNodePathName(getNodePath());
    if (0 != mPath.compare(0, path.size(), path))
      return;
    group.traverse(*this);
  }
  virtual void apply(Node& node)
  {
    if (mPath != Node::toNodePathName(getNodePath()))
      return;
    mNode = &node;
  }

  std::string mPath;
  Node* mNode;
};

Node*
System::getNode(const std::string& nodePathString)
{
  if (!mNode)
    return 0;
  NodeFinder nodeFinder(nodePathString);
  mNode->accept(nodeFinder);
  return nodeFinder.mNode;
}

const Node*
System::getNode(const std::string& nodePathString) const
{
  if (!mNode)
    return 0;
  NodeFinder nodeFinder(nodePathString);
  mNode->accept(nodeFinder);
  return nodeFinder.mNode;
}

} // namespace OpenFDM
