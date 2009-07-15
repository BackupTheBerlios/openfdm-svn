/* -*-c++-*- OpenFDM - Copyright (C) 2007-2009 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicContext_H
#define OpenFDM_MechanicContext_H

#include <list>
#include "Environment.h"
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
  MechanicContext(const Environment* environment);
  virtual ~MechanicContext();

  virtual const MechanicNode& getNode() const = 0;

  virtual void initDesignPosition();
  void initVelocities(const /*Init*/Task& task);
  virtual void init(const /*Init*/Task& task);
  virtual void velocities(const Task& task);
  virtual void articulation(const Task& task);
  virtual void accelerations(const Task& task);
  virtual void derivative(const Task&);
  virtual void update(const DiscreteTask& discreteTask);

  const Environment& getEnvironment() const
  { return *mEnvironment; }

private:
  MechanicContext(const MechanicContext&);
  MechanicContext& operator=(const MechanicContext&);

  SharedPtr<const Environment> mEnvironment;
};

class MechanicContextList : public std::list<SharedPtr<MechanicContext> > {
public:
  typedef std::list<SharedPtr<MechanicContext> > list_type;

  void initDesignPosition() const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->initDesignPosition();
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
