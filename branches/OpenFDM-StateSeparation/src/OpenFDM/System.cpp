/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#include "System.h"

#include "AbstractSystem.h"
#include "ConstNodeVisitor.h"
#include "Group.h"
#include "Interact.h"
#include "Joint.h"
#include "ModelInstance.h"
#include "MechanicInstance.h"
#include "NodeInstance.h"
#include "Object.h"
#include "RigidBody.h"
#include "RootJoint.h"
#include "SystemOutput.h"
#include "Task.h"

#include "Function.h"
#include "DoPri5.h"

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

    mODESolver = new DoPri5;
    mODESolver->setFunction(mContinousSystemFunction);
  }

  void appendModelInstance(OpenFDM::ModelInstance* modelInstance)
  {
    ModelContext* modelContext = &modelInstance->getNodeContext();
    SampleTime sampleTime = modelInstance->getSampleTime();

    // The init task contains them all
    mInitTask->mModelContextList[0].push_back(modelContext);
    
    // for now continous tasks take also all of them
    mContinousTask->appendStateValuesFromLeafContext(*modelContext);
    if (sampleTime.isContinous())
      mContinousTask->mModelContextList[0].push_back(modelContext);

    // Discrete tasks need special treatment
    if (sampleTime.isDiscrete()) {
      real_type realSampleTime = sampleTime.getSampleTime().getRealValue();
      for (unsigned i = 0; i < mDiscreteTaskList.size(); ++i) {
        if (!equal(mDiscreteTaskList[i]->getStepsize(), realSampleTime))
          continue;
        mDiscreteTaskList[i]->mModelContextList.push_back(modelContext);
        break;
      }
    }
  }

  void appendMechanicInstance(MechanicInstance* mechanicInstance)
  {
    MechanicContext* mechanicContext = &mechanicInstance->getNodeContext();
    // FIXME???
//     SampleTime sampleTime = modelInstance->getSampleTime();

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
  NodeInstanceCollector(const SampleTime& sampleTime) :
    mSampleTime(sampleTime),
    mBasicSampleTime(SampleTime::getContinous())
  { }

  struct _PortData;

  struct _PortConnectSet : public Referenced {
    bool setPortValue(PortValue* portValue)
    {
      while (!mParentPortData.empty()) {
        SharedPtr<_PortData> portData = mParentPortData.back().lock();
        mParentPortData.pop_back();
        if (!portData->setPortValue(portValue))
          return false;
      }
      return true;
    }
    std::vector<WeakPtr<_PortData> > mParentPortData;
  };

  struct _PortData : public WeakReferenced {
  public:
    _PortData(AbstractNodeInstance* nodeInstance, const PortInfo* portInfo) :
      mNodeInstance(nodeInstance),
      mPortValueCreator(true),
      mPortInfo(portInfo)
    {
      getOrCreatePortConnectSet();
    }

    void setPortConnectSet(_PortConnectSet* portSet)
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
        SharedPtr<_PortData> portData = mPortConnectSet->mParentPortData.back().lock();
        mPortConnectSet->mParentPortData.pop_back();
        if (portData == this)
          continue;
        portData->mPortConnectSet = portSet;
        portSet->mParentPortData.push_back(portData);
      }
      mPortConnectSet = portSet;
      mPortConnectSet->mParentPortData.push_back(this);
    }

    _PortConnectSet* getOrCreatePortConnectSet()
    {
      if (mPortConnectSet)
        return mPortConnectSet;
      setPortConnectSet(new _PortConnectSet);
      return mPortConnectSet;
    }

    bool addPortData(_PortData* portData)
    {
      if (getPortInfo()->getMaxConnects() <= mConnectedPorts.size())
        return false;
      mConnectedPorts.push_back(portData);
      setPortConnectSet(portData->getOrCreatePortConnectSet());
      return true;
    }

    bool connect(_PortData* portData)
    {
      if (getPortInfo()->getMaxConnects() <= mConnectedPorts.size())
        return false;
      if (!portData->addPortData(this))
        return false;
      mConnectedPorts.push_back(portData);
      setPortConnectSet(portData->getOrCreatePortConnectSet());
      return true;
    }

    const SharedPtr<const PortInfo>& getPortInfo() const
    { return mPortInfo; }

    bool setPortValue(PortValue* portValue)
    {
      if (!getPortInfo())
        return false;
      if (!mNodeInstance)
        return false;
      Log(Schedule, Debug3)
        << "setPortValue for port \"" << getPortInfo()->getName()
        << "\" is at: " << portValue << endl;
      // FIXME: move the set port value and accept port value into one call
      if (!getPortInfo()->acceptPortValue(portValue))
        return false;
      mNodeInstance->setPortValue(*getPortInfo(), portValue);
      return true;
    }

    void disablePortValueCreation()
    {
      mPortValueCreator = false;
    }

    void setProxyPortData(_PortData* proxyPortData)
    {
      mPortValueCreator = false;
      setPortConnectSet(proxyPortData->getOrCreatePortConnectSet());
    }

    bool createPortValue()
    {
      if (!mPortValueCreator)
        return true;
      if (mNodeInstance->getPortValue(*getPortInfo()))
        return true;
      SharedPtr<PortValue> portValue = getPortInfo()->newValue();
      if (!portValue)
        return true; // FIXME
      if (!mPortConnectSet->setPortValue(portValue))
        return false;
      return true;
    }

  private:
    bool mPortValueCreator;
    SharedPtr<AbstractNodeInstance> mNodeInstance;
    SharedPtr<const PortInfo> mPortInfo;
    std::vector<WeakPtr<_PortData> > mConnectedPorts;
    SharedPtr<_PortConnectSet> mPortConnectSet;
  };

  // Return true if this leaf directly depends on one of leafInstance outputs
  class _PortDataList : public Referenced {
  public:
    void setNodeInstance(AbstractNodeInstance* nodeInstance)
    {
      OpenFDMAssert(!mNodeInstance);
      mNodeInstance = nodeInstance;
      unsigned numPorts = nodeInstance->getNode().getNumPorts();
      mPortDataVector.resize(numPorts);
      for (unsigned i = 0; i < numPorts; ++i)
        mPortDataVector[i] = new _PortData(nodeInstance, nodeInstance->getNode().getPort(i));
    }
    
    _PortData* getPortData(const PortInfo& portInfo)
    { return mPortDataVector[portInfo.getIndex()]; }
    
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
    typedef std::vector<SharedPtr<_PortData> > PortDataVector;
    PortDataVector mPortDataVector;
    
    /// The AbstractNodeInstance having some way to reference the
    /// PortValues to the current connect information.
    SharedPtr<AbstractNodeInstance> mNodeInstance;
  };

  virtual void apply(const Node& node)
  { Log(Schedule, Error) << __PRETTY_FUNCTION__ << std::endl; }
  virtual void apply(const MechanicNode& node)
  { Log(Schedule, Error) << __PRETTY_FUNCTION__ << std::endl; }
  virtual void apply(const LeafNode& leaf)
  { Log(Schedule, Error) << __PRETTY_FUNCTION__ << std::endl; }
  virtual void apply(const LibraryNode& libraryNode)
  { Log(Schedule, Error) << __PRETTY_FUNCTION__ << std::endl; }

  _PortDataList* buildNodeContext(const Node& node)
  {
    OpenFDM::NodeInstance* nodeInstance;
    nodeInstance = new OpenFDM::NodeInstance(getNodePath(), mSampleTime, &node);
    _nodeInstanceList.push_back(nodeInstance);
    _PortDataList* portDataList;
    portDataList = getCurrentNodePortDataList();
    portDataList->setNodeInstance(nodeInstance);
    return portDataList;
  }

  virtual void apply(const GroupInterfaceNode& leaf)
  {
    _PortDataList* portDataList = buildNodeContext(leaf);
    OpenFDMAssert(leaf.getPort(0));
    _PortData* portData = portDataList->getPortData(*leaf.getPort(0));
    _groupPortDataMap[leaf.getExternalPortIndex()] = portData;
  }

  void allocPortData(AbstractNodeInstance* nodeInstance, const LeafNode& leaf)
  {
    _PortDataList* portDataList;
    portDataList = getCurrentNodePortDataList();
    portDataList->setNodeInstance(nodeInstance);
  }

  virtual void apply(const RootJoint& node)
  {
    // Need to stor the root nodes to build up the spanning tree for the
    // mechanical system here.
    OpenFDM::MechanicInstance* mechanicInstance = new MechanicInstance(getNodePath(), mSampleTime, &node);
    _nodeInstanceList.push_back(mechanicInstance);
    if (node.getNumPorts() == 1)
      _rootJointInstanceList.push_back(mechanicInstance);
    else
      _jointInstanceList.push_back(mechanicInstance);
    allocPortData(mechanicInstance, node);
  }
  virtual void apply(const Interact& node)
  {
    OpenFDM::MechanicInstance* mechanicInstance = new MechanicInstance(getNodePath(), mSampleTime, &node);
    _nodeInstanceList.push_back(mechanicInstance);
    _interactInstanceList.push_back(mechanicInstance);
    allocPortData(mechanicInstance, node);
  }
  virtual void apply(const RigidBody& node)
  {
    OpenFDM::MechanicInstance* mechanicInstance = new MechanicInstance(getNodePath(), mSampleTime, &node);
    _nodeInstanceList.push_back(mechanicInstance);
    allocPortData(mechanicInstance, node);
    // Make all rigid mechanic body links use the same link value
    _PortData* portData = 0;
    for (unsigned i = 0; i < node.getNumPorts(); ++i) {
      if (!node.getPort(i)->toMechanicLinkInfo())
        continue;
      if (portData) {
        mCurrentNodePortDataList->mPortDataVector[i]->setProxyPortData(portData);
      } else {
        portData = mCurrentNodePortDataList->mPortDataVector[i];
        portData->disablePortValueCreation();
      }
    }
  }
  virtual void apply(const Joint& node)
  {
    MechanicInstance* mechanicInstance = new MechanicInstance(getNodePath(), mSampleTime, &node);
    _nodeInstanceList.push_back(mechanicInstance);
    _jointInstanceList.push_back(mechanicInstance);
    allocPortData(mechanicInstance, node);
  }
  virtual void apply(const Model& node)
  {
    OpenFDM::ModelInstance* modelInstance = new OpenFDM::ModelInstance(getNodePath(), mSampleTime, &node);
    _nodeInstanceList.push_back(modelInstance);
    _modelInstanceList.push_back(modelInstance);
    allocPortData(modelInstance, node);
  }

  virtual void apply(const Group& group)
  {
    // Prepare a new leaf map for the child group
    _PortDataMap parentPortDataMap(group.getNumChildren());
    parentPortDataMap.swap(_portDataMap);

    // Get PortDataList indexed by group port index
    _ExternalGroupPortDataMap parentGroupPortDataMap(group.getNumPorts());
    parentGroupPortDataMap.swap(_groupPortDataMap);

    // End pushing external connection data

    // Now walk the children
    for (unsigned i = 0; i < group.getNumChildren(); ++i) {
      // push the sample time
      SampleTime sampleTime = mSampleTime;

      // our next node to traverse
      SharedPtr<const Node> node = group.getChild(i);

      // check what to do with sample times
      mSampleTime = node->getSampleTime();
      if (mSampleTime.isInherited())
        mSampleTime = sampleTime;
      else if (mSampleTime.isDiscrete()) {
        if (!mBasicSampleTime.isDiscrete())
          mBasicSampleTime = mSampleTime;
        else {
          Fraction a = mBasicSampleTime.getSampleTime();
          Fraction b = mSampleTime.getSampleTime();
          mBasicSampleTime = SampleTime(greatestCommonDivisor(a, b));
          OpenFDMAssert(mBasicSampleTime.isDiscrete());
        }
      }

      // Push the right per node port information struct
      SharedPtr<_PortDataList> parentNodePortDataList;
      parentNodePortDataList.swap(mCurrentNodePortDataList);

      mCurrentNodePortDataList = new _PortDataList;
      _portDataMap[i] = mCurrentNodePortDataList;
      _portDataListList.push_back(mCurrentNodePortDataList);

      // now traverse the child ...
      node->accept(*this);

      // Pop the per node port information struct
      parentNodePortDataList.swap(mCurrentNodePortDataList);

      // restore old group sample time
      mSampleTime = sampleTime;
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

    SharedPtr<_PortDataList> portDataList = buildNodeContext(group);

    // add group connect routings
    // merge child list into the global list of instances
    for (unsigned i = 0; i < group.getNumPorts(); ++i) {
      _PortData* portData = _groupPortDataMap[i];
      if (!portData) {
        Log(Schedule, Error) << "Internal Error: Cannot find internal port "
          "data for group external port!" << std::endl;
        continue;
      }

      // Allocate a new port data struct in the parent.
      _PortData* parentPortData = portDataList->getPortData(*group.getPort(i));
      parentPortData->setProxyPortData(portData);
      portData->setProxyPortData(parentPortData);
    }

    // Pop the per group port connect info
    parentGroupPortDataMap.swap(_groupPortDataMap);
    parentPortDataMap.swap(_portDataMap);
  }

  ////////////////////////////////////////////////////////////////////////////
  // The final list of Nodes we have in the simulation system
  OpenFDM::NodeInstanceList _nodeInstanceList;


  // The Models list, worthwhile for sorting
  OpenFDM::ModelInstanceList _modelInstanceList;
  // The mechanical system list, also for sorting
  OpenFDM::MechanicInstanceList _mechanicInstanceList;

  // The list of root nodes in the mechanical system. Will be a starting point
  // for sorting the tree of mechanical models downwards
  OpenFDM::MechanicInstanceList _rootJointInstanceList;
  OpenFDM::MechanicInstanceList _interactInstanceList;
  OpenFDM::MechanicInstanceList _jointInstanceList;

  ////////////////////////////////////////////////////////////////////////////
  // Used to map connections in groups ...
  typedef std::vector<SharedPtr<_PortDataList> > _PortDataMap;
  _PortDataMap _portDataMap;
  // Holds the PortDataList pointer indexed by parent groups port index
  typedef std::vector<SharedPtr<_PortData> > _ExternalGroupPortDataMap;
  _ExternalGroupPortDataMap _groupPortDataMap;
  // Just to hold references to all port data lists we have in the
  // simulation system. They are just needed during traversal for connect
  // information and to distribute port value pointers. If this list is not
  // built up the PortData values are deleted befor the PortValues are
  // distributed.
  typedef std::list<SharedPtr<_PortDataList> > _PortDataListList;
  _PortDataListList _portDataListList;

  // Current nodes sample time
  SampleTime mSampleTime;
  // past all the traversal, this contains the basic sample time of the
  // whole system. It is built up during traversal and has almost no meaning
  // until all models have be traversed.
  SampleTime mBasicSampleTime;

  // Here the miracle occurs.
  // The collected simulation nodes are packed into something that can be used
  // to simulate the system.
  AbstractSystem* buildSystem()
  {
    // Allocates and distributes the PortValues, is required for the sort
    // steps below
    if (!allocPortValues())
      return 0;
    // The MechanicNode instances are sorted to match the direct input property
    if (!sortMechanicList())
      return 0;
    // The model instances are sorted to match the direct input property
    if (!sortModelList())
      return 0;
    // Now that they are sorted, allocate the port sizes and with that
    // knowledge the state values.
    if (!allocModels())
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

    OpenFDM::ModelInstanceList::const_iterator i;
    for (i = _modelInstanceList.begin(); i != _modelInstanceList.end(); ++i) {
      discreteSystem->appendModelInstance(*i);
    }

    OpenFDM::MechanicInstanceList::const_iterator j;
    for (j = _mechanicInstanceList.begin();
         j != _mechanicInstanceList.end(); ++j) {
      discreteSystem->appendMechanicInstance(*j);
    }

    return discreteSystem.release();
  }

protected:
  // method to sort the leafs according to their dependency
  bool sortMechanicList()
  {
    if (_rootJointInstanceList.empty() &&
        (!_jointInstanceList.empty() || !_interactInstanceList.empty())) {
      Log(Schedule,Error)
        << "No root joint in System with mechanic components" << std::endl;
      return false;
    }

    // Start with all the roots in front of the list ...
    // FIXME: ensure that there is no loop here?
    _mechanicInstanceList.swap(_rootJointInstanceList);

    // Not the best algorithm, but for a first cut ...
    while (!_jointInstanceList.empty()) {
      OpenFDM::MechanicInstanceList nextLevelList;

      OpenFDM::MechanicInstanceList::iterator j;
      for (j = _mechanicInstanceList.begin();
           j != _mechanicInstanceList.end(); ++j) {
        OpenFDM::MechanicInstanceList::iterator i;
        for (i = _jointInstanceList.begin();
             i != _jointInstanceList.end();) {
        
          if ((*j)->isConnectedTo(*(*i))) {
            SharedPtr<OpenFDM::MechanicInstance> mechanicInstance = *i;
            nextLevelList.push_back(mechanicInstance);
            i = _jointInstanceList.erase(i);

            // Check if this current mechanic node does not reference
            // back into the already sorted models
            OpenFDM::MechanicInstanceList::const_iterator k;
            for (k = _mechanicInstanceList.begin();
                 k != _mechanicInstanceList.end(); ++k) {
              if (*k == *j)
                continue;
              if (mechanicInstance->isConnectedTo(*(*k))) {
                Log(Schedule,Error)
                  << "Detected closed kinematic loop: MechanicNode \""
                  << mechanicInstance->getNodeNamePath()
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
        OpenFDM::MechanicInstanceList::iterator i = j;
        for (++i; i != nextLevelList.end(); ++i) {
          if ((*j)->isConnectedTo(*(*i))) {
            Log(Schedule,Error)
              << "Detected closed kinematic loop: MechanicNode \""
              << (*j)->getNodeNamePath()
              << "\" is linked to MechanicNode \""
              << (*i)->getNodeNamePath() << "\"" << std::endl;
            return false;
          }
        }
      }
      

      for (j = nextLevelList.begin(); j != nextLevelList.end(); ++j) {
        _mechanicInstanceList.push_back(*j);
      }
    }

    // Interacts are always computed at the end of the list
    _mechanicInstanceList.splice(_mechanicInstanceList.end(),
                                      _interactInstanceList,
                                      _interactInstanceList.begin(),
                                      _interactInstanceList.end());
    
    Log(Schedule,Info) << "MechanicNode Schedule" << std::endl;
    OpenFDM::MechanicInstanceList::iterator i = _mechanicInstanceList.begin();
    for (; i != _mechanicInstanceList.end(); ++i) {
      Log(Schedule,Info)
        << "  MechanicNode \"" << (*i)->getNodeNamePath() << "\"" << std::endl;
    }

    return true;
  }

  // method to sort the leafs according to their dependency
  bool sortModelList()
  {
    OpenFDM::ModelInstanceList sortedModelInstanceList;
    while (!_modelInstanceList.empty()) {
      SharedPtr<OpenFDM::ModelInstance> modelInstance = _modelInstanceList.front();
      _modelInstanceList.pop_front();

      if (modelInstance->dependsOn(*modelInstance)) {
        Log(Schedule, Error)
          << "Self referencing direct dependency for Model \""
          << modelInstance->getNodeNamePath() << "\" detected!" << std::endl;
        return false;
      }

      OpenFDM::ModelInstanceList::iterator i;
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
    _PortDataListList::const_iterator i;
    for (i = _portDataListList.begin(); i != _portDataListList.end(); ++i) {
      if (!(*i)->allocAndConnectProviderPortValues())
        return false;
    }
    // check port values and report unconnected mandatory values.
    OpenFDM::NodeInstanceList::const_iterator j;
    for (j = _nodeInstanceList.begin(); j != _nodeInstanceList.end(); ++j) {
      const Node& node = (*j)->getNode();
      for (unsigned k = 0; k < node.getNumPorts(); ++k) {
        SharedPtr<const PortInfo> portInfo = node.getPort(k);
        if (portInfo->getOptional())
          continue;
        if (!(*j)->getPortValue(*portInfo)) {
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
    OpenFDM::ModelInstanceList::const_iterator i;
    for (i = _modelInstanceList.begin(); i != _modelInstanceList.end(); ++i) {
      if (!(*i)->getNodeContext().alloc()) {
        Log(Schedule, Error) << "Could not alloc for model \""
                             << (*i)->getNodeNamePath() << "\"" << endl;
        return false;
      }
    }

    OpenFDM::MechanicInstanceList::const_iterator j;
    for (j = _rootJointInstanceList.begin();
         j != _rootJointInstanceList.end(); ++j) {
      if (!(*j)->getNodeContext().alloc()) {
        Log(Schedule, Error) << "Could not alloc for MechanicNode \""
                             << (*j)->getNodeNamePath() << "\"" << endl;
        return false;
      }
    }
    for (j = _mechanicInstanceList.begin();
         j != _mechanicInstanceList.end(); ++j) {
      if (!(*j)->getNodeContext().alloc()) {
        Log(Schedule, Error) << "Could not alloc for MechanicNode \""
                             << (*j)->getNodeNamePath() << "\"" << endl;
        return false;
      }
    }
    return true;
  }

  _PortDataList* getCurrentNodePortDataList()
  {
    if (!mCurrentNodePortDataList)
      // will happen for the toplevel group node ..
      mCurrentNodePortDataList = new _PortDataList;
    return mCurrentNodePortDataList;
  }

private:
  SharedPtr<_PortDataList> mCurrentNodePortDataList;
};

BEGIN_OPENFDM_OBJECT_DEF(System, Object)
  END_OPENFDM_OBJECT_DEF

System::System(const std::string& name, Node* node) :
  Object(name),
  mNode(node),
  mSampleTime(SampleTime::getContinous())
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

bool
System::init(const real_type& t0)
{
  if (!mNode)
    return false;
  
  // Build up the lists required to run the model.
  NodeInstanceCollector nodeInstanceCollector(mSampleTime);
  mNode->accept(nodeInstanceCollector);
  
  mAbstractSystem = nodeInstanceCollector.buildSystem();
  if (!mAbstractSystem)
    return false;

  // Have something to run in our hands.
  // Not get the information required to reflect the system to the user.
  OpenFDM::NodeInstanceList::iterator i;
  for (i = nodeInstanceCollector._nodeInstanceList.begin();
       i != nodeInstanceCollector._nodeInstanceList.end(); ++i) {
    mNodeInstanceMap[(*i)->getNodePath()] = *i;
    mNodeInstanceList.push_back(*i);
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
  mNodeInstanceList.clear();
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

AbstractNodeInstance*
System::getNodeInstance(const NodePath& nodePath)
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

} // namespace OpenFDM
