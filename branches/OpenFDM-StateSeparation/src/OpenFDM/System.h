/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_System_H
#define OpenFDM_System_H

#include <string>

#include "ModelGroup.h"
#include "Environment.h"

namespace OpenFDM {

/// The System is the top \ref Model node.
/// It is derived from the \ref ModelGroup and additionally provides
/// algorithms to simulate and trim the whole system.

class TaskInfo;

class ODESolver;

class System : public ModelGroup {
  OPENFDM_OBJECT(System, ModelGroup);
public:
  /// Constructor, we need a name
  System(const std::string& name);
  virtual ~System(void);

  /// Double dispatch helper for the multibody system visitor
  virtual void accept(ModelVisitor& visitor);
  /// Double dispatch helper for the multibody system visitor
//   virtual void accept(ConstModelVisitor& visitor) const;

  /// Set the system to its initial state
  virtual bool init(void);

  /// Note that this is called *before* update() is called.
  virtual void output(const TaskInfo& taskInfo);
  /// Called whenever discrete states need to be updated.
  virtual void update(const TaskInfo& taskInfo);

  virtual void setState(const StateStream& state);
  virtual void getState(StateStream& state) const;
  virtual void getStateDeriv(StateStream& stateDeriv);

  virtual void setDiscreteState(const StateStream& state);
  virtual void getDiscreteState(StateStream& state) const;

  /// Simulate the system until the time tEnd
  bool simulate(real_type tEnd);

  /// Bring the system in an equilibrum state near the current state
  /// ...
  bool trim(void);

  /// Return the current simulation time, convenience function
  real_type getTime(void) const
  { return mTime; }

  /// Sets a timestepping algorithm for use with this system.
  void setTimestepper(ODESolver* timestepper);
  /// Return a const reference to the timestepping algorithm
  const ODESolver* getTimestepper(void) const { return mTimestepper; }
  /// Return a reference to the timestepping algorithm
  ODESolver* getTimestepper(void) { return mTimestepper; }

  /// FIXME Hmm, may be different ...
  /// May move into System ...
  void evalFunction(real_type t, const Vector& v, Vector& out);
  /// Compute the jacobian
  /// The default implementation computes a numeric approximation by finite
  /// differences
  void evalJacobian(real_type t, const Vector& state, Matrix& jac);

  Environment* getEnvironment(void) const
  { return mEnvironment; }

  /// Return the number of continous states
  unsigned getNumContinousStates(void) const
  { return mNumContinousStates; }
  /// Return the number of discrete states
  unsigned getNumDiscreteStates(void) const
  { return mNumDiscreteStates; }

protected:
  void setNumContinousStates(unsigned numContinousStates);
  void setNumDiscreteStates(unsigned numDiscreteStates);

private:
  /// The timestepper used to get time discrete approximate solutions to the
  /// continous system
  SharedPtr<ODESolver> mTimestepper;

  /// Hmm, need to think about this...
  typedef std::vector<TaskInfo> TaskList;

  TaskInfo mPerTimestepTask;
  TaskInfo mContinousTask;

  TaskList mDiscreteTaskList;
  unsigned mCurrentTaskNum;
  real_type mCurrentSliceTime;

  typedef std::vector<SharedPtr<Model> > ModelList;
  ModelList mDiscreteModelList;
  ModelList mContinousModelList;

  SharedPtr<Environment> mEnvironment;

  /// The actual simulation time for the system
  real_type mTime;

  /// The number of states in the whole System,
  /// might move into something like IntegrationGroup
  unsigned mNumContinousStates;
  unsigned mNumDiscreteStates;
};

} // namespace OpenFDM

#endif
