/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AbstractNodeInstance_H
#define OpenFDM_AbstractNodeInstance_H

#include <string>
#include "AbstractNodeContext.h"
#include "Assert.h"
#include "Node.h"
#include "SharedPtr.h"
#include "WeakReferenced.h"

namespace OpenFDM {

/// An Abstract NodeInstance represents an effective model node in a ready
/// to run System. You can access the Nodes Ports values for example.
/// This class is meant to show up in the user interface of this simulation.
class AbstractNodeInstance : public WeakReferenced {
public:
  AbstractNodeInstance(const NodePath& nodePath);
  virtual ~AbstractNodeInstance();

  /// The actual Node this AbstractNodeInstance stems from
  const Node& getNode() const
  { return getNodeContext().getNode(); }

  const NodePath& getNodePath() const { return mNodePath; }

//   /// Set the sample times this node will run on
//   void setSampleTimeSet(const SampleTimeSet& sampleTimeSet)
//   { mSampleTimeSet = sampleTimeSet; }
//   /// Get the sample times this node will run on
//   const SampleTimeSet& getSampleTimeSet() const
//   { return mSampleTimeSet; }

  PortValueList& getPortValueList()
  { return getNodeContext().getPortValueList(); }
  const PortValueList& getPortValueList() const
  { return getNodeContext().getPortValueList(); }

  std::string getNodeNamePath() const;

protected:
  /// The node context that belongs to this instance.
  virtual AbstractNodeContext& getNodeContext() = 0;
  virtual const AbstractNodeContext& getNodeContext() const = 0;

private:
  AbstractNodeInstance(const AbstractNodeInstance&);
  AbstractNodeInstance& operator=(const AbstractNodeInstance&);

//   /// The sample times this node will run on
//   SampleTimeSet mSampleTimeSet;

  NodePath mNodePath;
};

typedef std::list<SharedPtr<AbstractNodeInstance> > NodeInstanceList;

} // namespace OpenFDM

#endif
