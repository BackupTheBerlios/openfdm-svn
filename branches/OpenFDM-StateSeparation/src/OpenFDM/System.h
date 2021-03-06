/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_System_H
#define OpenFDM_System_H

#include <string>

#include "AbstractNodeInstance.h"
#include "Node.h"
#include "Object.h"
#include "SampleTime.h"
#include "SystemOutput.h"

namespace OpenFDM {

/// The System is the top \ref Node for a simulation system.
/// Provides algorithms to simulate and trim the whole system.

class AbstractSystem;
class Environment;
class SystemOutput;

class System : public Object {
  OPENFDM_OBJECT(System, Object);
public:
  System(const std::string& name, Node* node = 0);
  virtual ~System();

  SharedPtr<Node> getNode() { return mNode; }
  SharedPtr<const Node> getNode() const { return mNode; }
  void setNode(Node* node);

  /// The toplevel sample time of the system. Defaults to continous.
  /// Allowed values are continous and discrete sample times.
  /// Attemps to set an invalid value are ignored.
  const SampleTime& getSampleTime(void) const { return mSampleTime; }
  bool setSampleTime(const SampleTime& sampleTime);

  /// Access the environment functions bundle.
  void setEnvironment(Environment* environment);
  Environment* getEnvironment();
  const Environment* getEnvironment() const;

  bool init(const real_type& t0 = real_type(0));
  void clear();

  /// Simulate the system until the time t
  bool simulate(const real_type& t);

  /// Bring the system in an equilibrum state near the current state ...
  bool trim(void);

  /// Return the current simulation time, convenience function
  real_type getTime(void) const;

  /// Get node instances by their path within the system
  const AbstractNodeInstance* getNodeInstance(const NodePath& nodePath) const;

  void attach(SystemOutput* systemOutput);
  void detach(SystemOutput* systemOutput);

  /// Return the node that is at this particular path.
  Node* getNode(const std::string& nodePath);
  const Node* getNode(const std::string& nodePath) const;

private:
  class NodeInstanceCollector;
  class NodeFinder;

  SharedPtr<Node> mNode;
  SampleTime mSampleTime;

  SharedPtr<Environment> mEnvironment;

  SharedPtr<AbstractSystem> mAbstractSystem;

  typedef std::map<NodePath, SharedPtr<AbstractNodeInstance> > NodeInstanceMap;
  NodeInstanceMap mNodeInstanceMap;

  typedef std::list<SharedPtr<SystemOutput> > SystemOutputList;
  SystemOutputList mSystemOutputList;
};

} // namespace OpenFDM

#endif
