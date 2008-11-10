/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NodeInstance_H
#define OpenFDM_NodeInstance_H

#include "AbstractNodeInstance.h"

namespace OpenFDM {

class NodeInstance : public AbstractNodeInstance {
public:
  NodeInstance(const SampleTime& sampleTime,
               const Node* node);
  virtual ~NodeInstance();

  virtual const Node& getNode() const
  { return *mNode; }

  virtual const PortValue* getPortValue(const PortInfo& portInfo) const
  { return mPortValueList.getPortValue(portInfo); }
  virtual const NumericPortValue* getPortValue(const NumericPortInfo& portInfo) const
  { return mPortValueList.getPortValue(portInfo); }
  virtual const MechanicLinkValue* getPortValue(const MechanicLinkInfo& portInfo) const
  { return mPortValueList.getPortValue(portInfo); }

  /// Set port value for the given port.
  virtual void setPortValue(const PortInfo& portInfo, PortValue* portValue)
  { mPortValueList.setPortValue(portInfo.getIndex(), portValue); }

private:
  SharedPtr<const Node> mNode;

  PortValueList mPortValueList;
};

} // namespace OpenFDM

#endif
