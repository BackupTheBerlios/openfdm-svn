/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "System.h"

#include "AbstractSystem.h"
#include "ConstNodeVisitor.h"
#include "Group.h"
#include "ModelInstance.h"
#include "MechanicInstance.h"
#include "NodeInstance.h"
#include "Object.h"
#include "RootJoint.h"
#include "SystemLog.h"
#include "Task.h"

#include "Function.h"
#include "ExplicitEuler.h"

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

    mODESolver = new ExplicitEuler;
    mODESolver->setFunction(mContinousSystemFunction);
  }

  void appendModelContext(unsigned stride, ModelContext* modelContext)
  {
    // The init task contains them all
    mInitTask->mModelContextList[0].push_back(modelContext);
    // FIXME: decide which ones where ...
    mDiscreteTaskList[0]->mModelContextList.push_back(modelContext);
    mContinousTask->mModelContextList[0].push_back(modelContext);
    mContinousTask->appendStateValuesFromLeafContext(*modelContext);
  }

protected:
  virtual void initImplementation(const real_type& t)
  {
    mInitTask->init(t);

    if (mList.empty())
      return;

    mList.front().mSampleHit = t;
    discreteOutput(mList.front());

    // Set the state into the ode solver
    Vector v;
    mContinousTask->getStateValue(v);
    mODESolver->setState(v);
    mODESolver->setTime(t);
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
  }
  real_type getNextDiscreteSampleHit() const
  {
    OpenFDMAssert(getNextDiscreteSampleHitAlternate() == getValidityInterval().getEnd());
    return getValidityInterval().getEnd();
  }
  real_type getNextDiscreteSampleHitAlternate() const
  {
    if (mList.empty())
      return Limits<real_type>::max_value();
    return mList.front().mSampleHit;
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

  struct PortData : public WeakReferenced {
  public:
    PortData(AbstractNodeInstance* nodeInstance, const PortInfo* portInfo) :
      mNodeInstance(nodeInstance),
      mPortInfo(portInfo)
    { }
    virtual ~PortData()
    { }

    bool addPortData(PortData* portData)
    {
      if (getPortInfo()->getMaxConnects() <= mConnectedPorts.size())
        return false;
      mConnectedPorts.push_back(portData);
      return true;
    }

    bool connect(PortData* portData)
    {
      if (getPortInfo()->getMaxConnects() <= mConnectedPorts.size())
        return false;
      if (!portData->addPortData(this))
        return false;
      mConnectedPorts.push_back(portData);
      return true;
    }

    const SharedPtr<const PortInfo>& getPortInfo() const
    { return mPortInfo; }

    bool setLocalPortValue(PortValue* portValue)
    {
      if (!getPortInfo())
        return false;
      if (!mNodeInstance)
        return false;
      Log(Schedule, Debug3)
        << "setLocalPortValue for port \"" << getPortInfo()->getName()
        << "\" is at: " << portValue << endl;
      // FIXME: move the set port value and accept port value into one call
      if (!getPortInfo()->acceptPortValue(portValue))
        return false;
      unsigned index = getPortInfo()->getIndex();
      mNodeInstance->getPortValueList().setPortValue(index, portValue);
      return true;
    }

    bool setConnectedPortValues(PortValue* portValue)
    {
      Log(Schedule, Debug3)
        << "setConnectedPortValues for port \"" << getPortInfo()->getName()
        << "\" is at: " << portValue << endl;
      for (unsigned i = 0; i < mConnectedPorts.size(); ++i) {
        SharedPtr<PortData> portData = mConnectedPorts[i].lock();
        if (!portData)
          return false;
        if (!portData->setProxyPortValue(portValue))
          return false;
      }
      return setLocalPortValue(portValue);
    }

    bool setProxyPortValue(PortValue* portValue)
    {
      Log(Schedule, Debug3)
        << "setProxyPortValues for port \"" << getPortInfo()->getName()
        << "\" is at: " << portValue << endl;
      SharedPtr<PortData> portData = mProxyPortData.lock();
      if (portData) {
        if (!portData->setConnectedPortValues(portValue))
          return false;
      }
      return setLocalPortValue(portValue);
    }

    void setProxyPortData(PortData* proxyPortData)
    { mProxyPortData = proxyPortData; }

    virtual bool createPortValue()
    {
      if (mNodeInstance->getPortValueList().getPortValue(getPortInfo()->getIndex()))
        return true;
      if (mProxyPortData.lock())
        return true;
      SharedPtr<PortValue> portValue = getPortInfo()->newValue();
      if (!portValue)
        return true; // FIXME
      return setConnectedPortValues(portValue);
    }

  private:
    SharedPtr<AbstractNodeInstance> mNodeInstance;
    SharedPtr<const PortInfo> mPortInfo;
    std::vector<WeakPtr<PortData> > mConnectedPorts;
    WeakPtr<PortData> mProxyPortData;
  };

  // Return true if this leaf directly depends on one of leafInstance outputs
  class PortDataList : public Referenced {
  public:
    void setNodeInstance(AbstractNodeInstance* nodeInstance)
    {
      OpenFDMAssert(!mNodeInstance);
      mNodeInstance = nodeInstance;
      mPortDataVector.resize(nodeInstance->getNode().getNumPorts());
    }
    
    PortData* newPortData(const PortInfo* portInfo)
    {
      PortData* portData = new PortData(mNodeInstance, portInfo);
      mPortDataVector[portInfo->getIndex()] = portData;
      return portData;
    }
    
    bool allocAndConnectProviderPortValues()
    {
      for (unsigned i = 0; i < mPortDataVector.size(); ++i) {
        Log(Schedule, Debug3) << "Try to to allocate port value \""
                               << mPortDataVector[i]->getPortInfo()->getName()
                               << "\" of \"" << mNodeInstance->getNodeNamePath()
                               << "\"" << endl;
        if (!mPortDataVector[i]->createPortValue()) {
          Log(Schedule, Error) << "Failed to allocate port value \""
                               << mPortDataVector[i]->getPortInfo()->getName()
                               << "\" of \"" << mNodeInstance->getNodeNamePath()
                               << "\".\nAborting!" << endl;

          return false;
        }
      }
      return true;
    }

    /// The vector of per port connect information
    typedef std::vector<SharedPtr<PortData> > PortDataVector;
    PortDataVector mPortDataVector;
    
    /// The AbstractNodeInstance having some way to reference the
    /// PortValues to the current connect information.
    SharedPtr<AbstractNodeInstance> mNodeInstance;
  };

  virtual void apply(const Node& node)
  { Log(Schedule, Error) << __PRETTY_FUNCTION__ << std::endl; }
  virtual void apply(const LeafNode& leaf)
  { Log(Schedule, Error) << __PRETTY_FUNCTION__ << std::endl; }
  virtual void apply(const LibraryNode& libraryNode)
  { Log(Schedule, Error) << __PRETTY_FUNCTION__ << std::endl; }

  PortDataList* buildNodeContext(const Node& node)
  {
    NodeInstance* nodeInstance;
    nodeInstance = new NodeInstance(getNodePath(), &node);
    _nodeInstanceList.push_back(nodeInstance);
    PortDataList* portDataList;
    portDataList = getCurrentNodePortDataList();
    portDataList->setNodeInstance(nodeInstance);
    return portDataList;
  }

  virtual void apply(const GroupInterfaceNode& leaf)
  {
    PortDataList* portDataList = buildNodeContext(leaf);
    OpenFDMAssert(leaf.getPort(0));
    PortData* portData = portDataList->newPortData(leaf.getPort(0));
    _groupPortDataMap[leaf.getExternalPortIndex()] = portData;
  }

  void allocPortData(AbstractNodeInstance* nodeInstance, const LeafNode& leaf)
  {
    PortDataList* portDataList;
    portDataList = getCurrentNodePortDataList();
    portDataList->setNodeInstance(nodeInstance);

    for (unsigned i = 0; i < leaf.getNumPorts(); ++i)
      portDataList->newPortData(leaf.getPort(i));
  }

  virtual void apply(const RootJoint& node)
  {
    // Need to stor the root nodes to build up the spanning tree for the
    // mechanical system here.
    MechanicInstance* mechanicInstance = new MechanicInstance(getNodePath(), &node);
    _nodeInstanceList.push_back(mechanicInstance);
//     _mechanicInstanceList.push_back(mechanicInstance);
    _rootJointInstanceList.push_back(mechanicInstance);
    allocPortData(mechanicInstance, node);
  }
  virtual void apply(const MechanicNode& node)
  {
    MechanicInstance* mechanicInstance = new MechanicInstance(getNodePath(), &node);
    _nodeInstanceList.push_back(mechanicInstance);
    _mechanicInstanceList.push_back(mechanicInstance);
    allocPortData(mechanicInstance, node);
  }
  virtual void apply(const Model& node)
  {
    ModelInstance* modelInstance = new ModelInstance(getNodePath(), &node);
    _nodeInstanceList.push_back(modelInstance);
    _modelInstanceList.push_back(modelInstance);
    allocPortData(modelInstance, node);
  }

  virtual void apply(const Group& group)
  {
    // Prepare a new leaf map for the child group
    PortDataMap parentPortDataMap(group.getNumChildren());
    for (unsigned i = 0; i < group.getNumChildren(); ++i) {
      PortDataList* portDataList;
      portDataList = new PortDataList;
      parentPortDataMap[i] = portDataList;
      _portDataListList.push_back(portDataList);
    }
    parentPortDataMap.swap(_portDataMap);

    // Get PortDataList indexed by group port index
    ExternalGroupPortDataMap parentGroupPortDataMap(group.getNumPorts());
    parentGroupPortDataMap.swap(_groupPortDataMap);

    // End pushing external connection data

    // Now walk the children
    for (unsigned i = 0; i < group.getNumChildren(); ++i) {
      // Push the right per node port information struct
      SharedPtr<PortDataList> parentNodePortDataList;
      parentNodePortDataList.swap(mCurrentNodePortDataList);
      mCurrentNodePortDataList = _portDataMap[i];

      group.getChild(i)->accept(*this);

      // Pop the per node port information struct
      parentNodePortDataList.swap(mCurrentNodePortDataList);
    }

    // Apply the group internal connections to the instances
    unsigned numConnects = group.getNumConnects();
    for (unsigned i = 0; i < numConnects; ++i) {
      unsigned nodeIndex0 = group.getConnectNodeIndex0(i);
      if (nodeIndex0 == ~0u) {
        Log(Schedule, Error)
          << "Cannot find node from nodeId" << std::endl;
        continue;
      }
      unsigned nodeIndex1 = group.getConnectNodeIndex1(i);
      if (nodeIndex1 == ~0u) {
        Log(Schedule, Error)
          << "Cannot find node from nodeId" << std::endl;
        continue;
      }

      SharedPtr<const PortInfo> portInfo0 = group.getConnectPortInfo0(i);
      if (!portInfo0) {
        Log(Schedule, Error) << "Cannot find provider Port data node "
                  << group.getChild(nodeIndex0)->getName() << std::endl;
        continue;
      }
      SharedPtr<const PortInfo> portInfo1 = group.getConnectPortInfo1(i);
      if (!portInfo1) {
        Log(Schedule, Error) << "Cannot find acceptor Port data node "
                  << group.getChild(nodeIndex1)->getName() << std::endl;
        continue;
      }

      unsigned portInfoIndex0 = portInfo0->getIndex();
      unsigned portInfoIndex1 = portInfo1->getIndex();
      if (!_portDataMap[nodeIndex1]->mPortDataVector[portInfoIndex1]->
          connect(_portDataMap[nodeIndex0]->mPortDataVector[portInfoIndex0]))
        Log(Schedule, Error) << "Internal Error: Cannot connect ports that"
          " appeared to be compatible before." << std::endl;
    }

    SharedPtr<PortDataList> portDataList = buildNodeContext(group);

    // add group connect routings
    // merge child list into the global list of instances
    for (unsigned i = 0; i < group.getNumPorts(); ++i) {
      PortData* portData = _groupPortDataMap[i];
      if (!portData) {
        Log(Schedule, Error) << "Internal Error: Cannot find internal port "
          "data for group external port!" << std::endl;
        continue;
      }

      // Allocate a new port data struct in the parent.
      PortData* parentPortData = portDataList->newPortData(group.getPort(i));
      parentPortData->setProxyPortData(portData);
      portData->setProxyPortData(parentPortData);
    }

    // Pop the per group port connect info
    parentGroupPortDataMap.swap(_groupPortDataMap);
    parentPortDataMap.swap(_portDataMap);
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
  typedef std::vector<SharedPtr<PortDataList> > PortDataMap;
  PortDataMap _portDataMap;
  // Holds the PortDataList pointer indexed by parent groups port index
  typedef std::vector<SharedPtr<PortData> > ExternalGroupPortDataMap;
  ExternalGroupPortDataMap _groupPortDataMap;
  // Just to hold references to all port data lists we have in the
  // simulation system. They are just needed during traversal for connect
  // information and to distribute port value pointers. If this list is not
  // built up the PortData values are deleted befor the PortValues are
  // distributed.
  typedef std::list<SharedPtr<PortDataList> > PortDataListList;
  PortDataListList _portDataListList;

  // Here the miracle occurs.
  // The collected simulation nodes are packed into something that can be used
  // to simulate the system.
  AbstractSystem* buildSystem()
  {
    // Allocates and distributes the PortValues, is required for the sort
    // steps below
    if (!allocPortValues())
      return 0;
    // The model instances are sorted to match the direct input property
    if (!sortModelList())
      return 0;
    // Now that they are sorted, allocate the port sizes and with that
    // knowledge the state values.
    if (!allocModels())
      return 0;

    // Now the system is ready for state initialization and execution
    // Build up te abstract system and return that

    // For the first cut, assume many things like basic step size and such ...
    SharedPtr<DiscreteSystem> discreteSystem;
    discreteSystem = new DiscreteSystem(0.01, 1);

    ModelInstanceList::const_iterator i;
    for (i = _modelInstanceList.begin(); i != _modelInstanceList.end(); ++i) {
      discreteSystem->appendModelContext(1, &(*i)->getNodeContext());
    }

    return discreteSystem.release();
  }

protected:
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
        if (!(*i)->dependsOn(*modelInstance))
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
          if (!modelInstance->dependsOn(*(*i)))
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
    // alloc port values
    PortDataListList::const_iterator i;
    for (i = _portDataListList.begin(); i != _portDataListList.end(); ++i) {
      if (!(*i)->allocAndConnectProviderPortValues())
        return false;
    }
    // check port values and report unconnected mandatory values.
    ModelInstanceList::const_iterator j;
    for (j = _modelInstanceList.begin(); j != _modelInstanceList.end(); ++j) {
      const Node& node = (*j)->getNode();
      for (unsigned k = 0; k < node.getNumPorts(); ++k) {
        SharedPtr<const PortInfo> portInfo = node.getPort(k);
        if (portInfo->getOptional())
          continue;
        if (!(*j)->getPortValueList().getPortValue(k)) {
          Log(Schedule, Error) << "Mandatory port value for port \""
                               << portInfo->getName() << "\" for model \""
                               << (*j)->getNodeNamePath()
                               << "\" is not connected!" << endl;
          return false;
        }
      }
    }
    return true;
  }

  bool allocModels()
  {
    ModelInstanceList::const_iterator i;
    for (i = _modelInstanceList.begin(); i != _modelInstanceList.end(); ++i) {
      if (!(*i)->getNodeContext().alloc()) {
        Log(Schedule, Error) << "Could not alloc for model \""
                             << (*i)->getNodeNamePath() << "\"" << endl;
        return false;
      }
    }
    return true;
  }

  PortDataList* getCurrentNodePortDataList()
  {
    if (!mCurrentNodePortDataList)
      // will happen for the toplevel group node ..
      mCurrentNodePortDataList = new PortDataList;
    return mCurrentNodePortDataList;
  }

private:
  SharedPtr<PortDataList> mCurrentNodePortDataList;
};

BEGIN_OPENFDM_OBJECT_DEF(System, Object)
  END_OPENFDM_OBJECT_DEF

System::System(const std::string& name, Node* node) :
  Object(name),
  mNode(node)
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
System::init()
{
  if (!mNode)
    return false;
  
  // Build up the lists required to run the model.
  NodeInstanceCollector nodeInstanceCollector;
  mNode->accept(nodeInstanceCollector);
  
  mAbstractSystem = nodeInstanceCollector.buildSystem();
  if (!mAbstractSystem)
    return false;

  // Have something to run in our hands.
  // Not get the information required to reflect the system to the user.
  NodeInstanceList::iterator i;
  for (i = nodeInstanceCollector._nodeInstanceList.begin();
       i != nodeInstanceCollector._nodeInstanceList.end(); ++i) {
    mNodeInstanceMap[(*i)->getNodePath()] = *i;
    mNodeInstanceList.push_back(*i);
  }

  SystemLogList::const_iterator j;
  for (j = mSystemLogList.begin(); j != mSystemLogList.end(); ++j)
    (*j)->attachTo(this);
  
  // Hmm, really here???
  mAbstractSystem->init(0);

  return true;
}

void
System::clear()
{
  mAbstractSystem = 0;
  mNodeInstanceList.clear();
  mNodeInstanceMap.clear();

  SystemLogList::const_iterator i;
  for (i = mSystemLogList.begin(); i != mSystemLogList.end(); ++i)
    (*i)->attachTo(0);
}

/// Simulate the system until the time tEnd
bool
System::simulate(const real_type& t)
{
  if (!mAbstractSystem)
    return false;
  mAbstractSystem->outputAt(t);

  SystemLogList::const_iterator i;
  for (i = mSystemLogList.begin(); i != mSystemLogList.end(); ++i)
    (*i)->output(t);

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

AbstractNodeInstance*
System::getNodeInstance(const NodePath& nodePath)
{
  NodeInstanceMap::const_iterator i = mNodeInstanceMap.find(nodePath);
  if (i == mNodeInstanceMap.end())
    return 0;
  return i->second;
}

void
System::attach(SystemLog* systemLog)
{
  mSystemLogList.push_back(systemLog);
  if (!mAbstractSystem)
    return;
  systemLog->attachTo(this);
}

void
System::detach(SystemLog* systemLog)
{
  SystemLogList::iterator i = mSystemLogList.begin();
  while (i != mSystemLogList.end()) {
    if (*i == systemLog) {
      i = mSystemLogList.erase(i);
      if (mAbstractSystem)
        systemLog->attachTo(0);
    } else
      ++i;
  }
}

} // namespace OpenFDM
