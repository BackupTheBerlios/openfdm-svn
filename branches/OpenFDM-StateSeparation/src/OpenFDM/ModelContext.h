/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ModelContext_H
#define OpenFDM_ModelContext_H

#include <list>
#include "SharedPtr.h"
#include "LeafContext.h"
#include "AbstractModel.h"

namespace OpenFDM {

class Task;
class ContinousTask;
class DiscreteTask;
class InitTask;

//// This one is used to execute the simulation system
class ModelContext : public LeafContext {
public:
  ModelContext() {}
  virtual ~ModelContext();

  virtual const AbstractModel& getNode() const = 0;

  virtual void initOutput(const /*Init*/Task& task) = 0;
  virtual void output(const Task& task) = 0;
  virtual void update(const DiscreteTask& discreteTask) = 0;
  virtual void derivative(const Task&) = 0;

  /// might vanish???
  PortValueList& getPortValueList()
  { return mPortValueList; }

private:
  ModelContext(const ModelContext&);
  ModelContext& operator=(const ModelContext&);
};

class ModelContextList : public std::list<SharedPtr<ModelContext> > {
public:
  typedef std::list<SharedPtr<ModelContext> > list_type;

  void initOutput(const /*Init*/Task& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->initOutput(task);
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
  void derivative(const Task& task) const
  {
    for (list_type::const_iterator i = begin(); i != end(); ++i)
      (*i)->derivative(task);
  }
};

} // namespace OpenFDM

#endif
