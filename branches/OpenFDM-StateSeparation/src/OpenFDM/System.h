/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_System_H
#define OpenFDM_System_H

#include <string>

#include "AbstractNodeInstance.h"
#include "Node.h"
#include "Object.h"
#include "SampleTime.h"

namespace OpenFDM {

/// The System is the top \ref Node for a simulation system.
/// Provides algorithms to simulate and trim the whole system.

class AbstractSystem;
class SystemLog;

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

  bool init();
  void clear();

  /// Simulate the system until the time tEnd
  bool simulate(const real_type& t);

  /// Bring the system in an equilibrum state near the current state ...
  bool trim(void);

  /// Return the current simulation time, convenience function
  real_type getTime(void) const;

  /// Get the whole NodeInstance list
  const ConstNodeInstanceList& getNodeInstanceList() const
  { return mNodeInstanceList; }

  /// Get node instances by their path within the system
  const AbstractNodeInstance* getNodeInstance(const NodePath& nodePath) const;
  AbstractNodeInstance* getNodeInstance(const NodePath& nodePath);

  void attach(SystemLog* systemLog);
  void detach(SystemLog* systemLog);

private:
  class NodeInstanceCollector;

  SharedPtr<Node> mNode;
  SampleTime mSampleTime;

  SharedPtr<AbstractSystem> mAbstractSystem;
  ConstNodeInstanceList mNodeInstanceList;
  NodeInstanceMap mNodeInstanceMap;

  typedef std::list<SharedPtr<SystemLog> > SystemLogList;
  SystemLogList mSystemLogList;
};

} // namespace OpenFDM

#endif
