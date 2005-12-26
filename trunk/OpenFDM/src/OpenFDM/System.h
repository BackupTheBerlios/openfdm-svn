/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_System_H
#define OpenFDM_System_H

#include <string>

#include "ModelGroup.h"
#include "ODESolver.h"

namespace OpenFDM {

/// The System is the top \ref Model node.
/// It is derived from the \ref ModelGroup and additionally provides
/// algorithms to simulate and trim the whole system.

class TaskInfo;

class System : public ModelGroup {
public:
  /// Constructor, we need a name
  System(const std::string& name);
  virtual ~System(void);

  /// Set the system to its initial state
  virtual bool init(void);

  /// Note that there are still these routines available
//   virtual void output();
//   virtual void update(real_type dt);

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

  virtual Environment* getEnvironment(void) const;

private:
  /// The timestepper used to get time discrete approximate solutions to the
  /// continous system
  SharedPtr<ODESolver> mTimestepper;

  /// Hmm, need to think about this...
  typedef std::vector<TaskInfo> TaskList;
  TaskList mDiscreteTaskList;
  unsigned mCurrentTaskNum;
  real_type mCurrentSliceTime;

  SharedPtr<Environment> mEnvironment;

  /// The actual simulation time for the system
  real_type mTime;
};

} // namespace OpenFDM

#endif
