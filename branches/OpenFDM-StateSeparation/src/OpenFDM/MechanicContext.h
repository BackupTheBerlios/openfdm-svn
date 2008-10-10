/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicContext_H
#define OpenFDM_MechanicContext_H

#include <list>
#include "SharedPtr.h"
#include "LeafContext.h"
#include "MechanicNode.h"

namespace OpenFDM {

class Task;
class ContinousTask;
class DiscreteTask;
class InitTask;

class MechanicContext : public LeafContext {
public:
  MechanicContext(const MechanicNode* mechanicNode);
  virtual ~MechanicContext();

  virtual const MechanicNode& getNode() const;

  bool alloc()
  { return mMechanicNode->alloc(*this); }
  void init(const /*Init*/Task& task)
  { mMechanicNode->init(task, mDiscreteState, mContinousState, mPortValueList); }

  void velocities(const Task&)
  { mMechanicNode->velocity(mContinousState, mPortValueList); }
  void articulation(const Task&)
  { mMechanicNode->articulation(mContinousState, mPortValueList); }
  void accelerations(const Task&)
  { }

  void derivative(const Task&)
  { mMechanicNode->derivative(mDiscreteState, mContinousState, mPortValueList,
                              mContinousStateDerivative); }
 
//   void outputVelocities()
//   { }
//   void outputAcceperation()
//   { }

  void update(const DiscreteTask& discreteTask)
  {
    mMechanicNode->update(discreteTask, mDiscreteState,
                          mContinousState, mPortValueList);
  }

  bool isConnectedTo(const MechanicContext& mechanicContext) const;

private:
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
  void init(const /*Init*/Task& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->init(task);
  }
  void velocities(const Task& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->velocities(task);
  }
  void articulation(const Task& task) const
  {
    // Note that this list is traversed from the mechanic leafs to the root
    for (list_type::const_reverse_iterator i = rbegin(); i != rend(); ++i)
      (*i)->articulation(task);
  }
  void accelerations(const Task& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->accelerations(task);
  }
  void update(const DiscreteTask& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->update(task);
  }
  void derivative(const Task& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->derivative(task);
  }
};

} // namespace OpenFDM

#endif
