/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "Interact.h"

#include "ConstNodeVisitor.h"
#include "MechanicContext.h"
#include "NodeVisitor.h"

namespace OpenFDM {

class Interact::Context : public MechanicContext {
public:
  Context(const Interact* interact) : mInteract(interact) {}
  virtual ~Context() {}

  virtual const Interact& getNode() const
  { return *mInteract; }

  virtual bool alloc()
  { if (!allocStates()) return false; return mInteract->alloc(*this); }
  virtual void initVelocities(const /*Init*/Task& task)
  {
    mInteract->init(task, mDiscreteState, mContinousState, mPortValueList);
    mInteract->velocity(task, mContinousState, mPortValueList);
  }

  virtual void velocities(const Task& task)
  {
    mInteract->velocity(task, mContinousState, mPortValueList);
  }
  virtual void articulation(const Task& task)
  {
    mInteract->articulation(task, mContinousState, mPortValueList, hIh);
  }
  virtual void accelerations(const Task& task)
  {
    mInteract->acceleration(task, mContinousState, mPortValueList, hIh, velDot);
  }

  virtual void derivative(const Task&)
  {
    mInteract->derivative(mDiscreteState, mContinousState, mPortValueList,
                              velDot, mContinousStateDerivative);
  }
 
  virtual void update(const DiscreteTask&)
  { }

private:
  // Stores some values persistent accross velocity/articulation/acceleration
  Matrix hIh;
  Vector velDot;

  SharedPtr<const Interact> mInteract;
};

BEGIN_OPENFDM_OBJECT_DEF(Interact, MechanicNode)
  END_OPENFDM_OBJECT_DEF

Interact::Interact(const std::string& name) :
  MechanicNode(name)
{
}

Interact::~Interact()
{
}

void
Interact::accept(NodeVisitor& visitor)
{
  visitor.handleNodePathAndApply(this);
}

void
Interact::accept(ConstNodeVisitor& visitor) const
{
  visitor.handleNodePathAndApply(this);
}

MechanicContext*
Interact::newMechanicContext(PortValueList& portValueList) const
{
  SharedPtr<MechanicContext> context = new Context(this);
  for (unsigned i = 0; i < getNumPorts(); ++i) {
    PortValue* portValue = portValueList.getPortValue(i);
    if (!portValue) {
      Log(Model, Error) << "No port value given for model \"" << getName()
                        << "\" and port \"" << getPort(i)->getName()
                        << "\"" << endl;
      return false;
    }
    context->setPortValue(*getPort(i), portValue);
  }
  if (!context->alloc()) {
    Log(Model, Warning) << "Could not alloc for model \""
                        << getName() << "\"" << endl;
    return false;
  }
  return context.release();
}

} // namespace OpenFDM
