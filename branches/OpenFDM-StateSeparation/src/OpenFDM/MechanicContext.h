/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicContext_H
#define OpenFDM_MechanicContext_H

#include <list>
#include "LeafContext.h"
#include "MechanicNode.h"
#include "SharedPtr.h"
#include "Transform.h"

namespace OpenFDM {

class Task;
class ContinousTask;
class DiscreteTask;
class InitTask;

class MechanicContext : public LeafContext {
public:
  MechanicContext() {}
  virtual ~MechanicContext();

  virtual const MechanicNode& getNode() const = 0;

  virtual bool alloc() = 0;
  virtual void initVelocities(const /*Init*/Task& task) = 0;
  virtual void velocities(const Task& task) = 0;
  virtual void articulation(const Task& task) = 0;
  virtual void accelerations(const Task& task) = 0;
  virtual void derivative(const Task&) = 0;
  virtual void update(const DiscreteTask& discreteTask) = 0;

private:
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
  void initVelocities(const /*Init*/Task& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->initVelocities(task);
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
