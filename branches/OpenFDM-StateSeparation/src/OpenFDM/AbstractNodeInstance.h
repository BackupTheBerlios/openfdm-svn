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
  AbstractNodeInstance(const SampleTime& sampleTime);
  virtual ~AbstractNodeInstance();

  /// The actual Node this AbstractNodeInstance stems from
  virtual const Node& getNode() const = 0;

  /// Get the sample time this node will run on
  const SampleTime& getSampleTime() const
  { return mSampleTime; }

  /// Access port values by the PortInfo values
  virtual const PortValue*
  getPortValue(const PortInfo& portInfo) const = 0;
  virtual const NumericPortValue*
  getPortValue(const NumericPortInfo& portInfo) const = 0;
  virtual const MechanicLinkValue*
  getPortValue(const MechanicLinkInfo& portInfo) const = 0;

  /// Set port value for the given port.
  // FIXME, must vanish ...
  virtual void setPortValue(const PortInfo& portInfo, PortValue* portValue) = 0;

private:
  AbstractNodeInstance(const AbstractNodeInstance&);
  AbstractNodeInstance& operator=(const AbstractNodeInstance&);

  /// The sample times this node will run on
  const SampleTime mSampleTime;
};

class LeafInstance : public AbstractNodeInstance {
public:
  LeafInstance(const SampleTime& sampleTime, AbstractNodeContext* context) :
    AbstractNodeInstance(sampleTime),
    mNodeContext(context)
  { }
  virtual ~LeafInstance() {}

  /// The actual Node this AbstractLeafInstance stems from
  virtual const Node& getNode() const
  { return mNodeContext->getNode(); }

  /// Access port values by the PortInfo values
  virtual const PortValue*
  getPortValue(const PortInfo& portInfo) const
  { return mNodeContext->getPortValueList().getPortValue(portInfo); }
  virtual const NumericPortValue*
  getPortValue(const NumericPortInfo& portInfo) const
  { return mNodeContext->getPortValueList().getPortValue(portInfo); }
  virtual const MechanicLinkValue*
  getPortValue(const MechanicLinkInfo& portInfo) const
  { return mNodeContext->getPortValueList().getPortValue(portInfo); }

  /// Set port value for the given port.
  virtual void setPortValue(const PortInfo& portInfo, PortValue* portValue)
  { mNodeContext->setPortValue(portInfo, portValue); }

private:
  LeafInstance(const LeafInstance&);
  LeafInstance& operator=(const LeafInstance&);

  SharedPtr<AbstractNodeContext> mNodeContext;
};

} // namespace OpenFDM

#endif
