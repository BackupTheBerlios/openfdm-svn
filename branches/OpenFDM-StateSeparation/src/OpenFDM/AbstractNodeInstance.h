/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AbstractNodeInstance_H
#define OpenFDM_AbstractNodeInstance_H

#include <string>
#include <list>
#include <map>
#include "AbstractNodeContext.h"
#include "Assert.h"
#include "Node.h"
#include "SampleTime.h"
#include "SharedPtr.h"
#include "WeakReferenced.h"

namespace OpenFDM {

/// An Abstract NodeInstance represents an effective model node in a ready
/// to run System. You can access the Nodes Ports values for example.
/// This class is meant to show up in the user interface of this simulation.
class AbstractNodeInstance : public WeakReferenced {
public:
  AbstractNodeInstance(const NodePath& nodePath, const SampleTime& sampleTime);
  virtual ~AbstractNodeInstance();

  /// The actual Node this AbstractNodeInstance stems from
  const Node& getNode() const
  { return getNodeContext().getNode(); }

  /// The node path leading to this instance.
  const NodePath& getNodePath() const
  { return mNodePath; }
  /// String representation of the node path.
  std::string getNodeNamePath() const;

  /// Get the sample time this node will run on
  const SampleTime& getSampleTime() const
  { return mSampleTime; }

  PortValueList& getPortValueList()
  { return getNodeContext().getPortValueList(); }
  const PortValueList& getPortValueList() const
  { return getNodeContext().getPortValueList(); }

protected:
  /// The node context that belongs to this instance.
  virtual AbstractNodeContext& getNodeContext() = 0;
  virtual const AbstractNodeContext& getNodeContext() const = 0;

private:
  AbstractNodeInstance(const AbstractNodeInstance&);
  AbstractNodeInstance& operator=(const AbstractNodeInstance&);

  /// The sample times this node will run on
  const SampleTime mSampleTime;

  const NodePath mNodePath;
};

typedef std::list<SharedPtr<AbstractNodeInstance> > NodeInstanceList;
typedef std::list<SharedPtr<const AbstractNodeInstance> > ConstNodeInstanceList;
typedef std::map<NodePath, SharedPtr<AbstractNodeInstance> > NodeInstanceMap;

} // namespace OpenFDM

#endif
