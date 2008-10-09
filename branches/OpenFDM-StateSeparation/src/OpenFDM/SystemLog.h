/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SystemLog_H
#define OpenFDM_SystemLog_H

#include <OpenFDM/System.h>
#include <OpenFDM/ConstNodeVisitor.h>
#include <OpenFDM/NodeInstance.h>

namespace OpenFDM {

class SystemLog : public ConstNodeVisitor {
public:
  virtual ~SystemLog() {}

  virtual void output(const real_type& t) = 0;

  void attachTo(const System* system)
  {
    mSystem = system;
    if (!system)
      return;
    system->getNode()->accept(*this);
  }

  virtual void apply(const PortInfo* portInfo, const PortValue* portValue)
  { }
  virtual void apply(const PortInfo* portInfo,
                     const NumericPortValue* numericPortValue)
  { apply(portInfo, static_cast<const PortValue*>(numericPortValue)); }
  virtual void apply(const PortInfo* portInfo,
                     const MechanicPortValue* mechanicPortValue)
  { apply(portInfo, static_cast<const PortValue*>(mechanicPortValue)); }

protected:
  const AbstractNodeInstance* getNodeInstance(const NodePath& nodePath) const
  {
    SharedPtr<const System> system = mSystem.lock();
    if (!system)
      return 0;
    return system->getNodeInstance(nodePath);
  }
  void appendPortValues(const Node&)
  {
    const AbstractNodeInstance* nodeInstance = getNodeInstance(getNodePath());
    if (!nodeInstance)
      return;
    appendPortValues(*nodeInstance);
  }
  void appendPortValues(const AbstractNodeInstance& nodeInstance)
  {
    unsigned numPorts = nodeInstance.getNode().getNumPorts();
    for (unsigned i = 0; i < numPorts; ++i) {
      const PortValue* portValue;
      portValue = nodeInstance.getPortValueList().getPortValue(i);
      const NumericPortValue* npv = portValue->toNumericPortValue();
      if (npv) {
        apply(nodeInstance.getNode().getPort(i), npv);
        continue;
      }

      const MechanicPortValue* mpv = portValue->toMechanicPortValue();
      if (npv) {
        apply(nodeInstance.getNode().getPort(i), mpv);
        continue;
      }

      apply(nodeInstance.getNode().getPort(i), portValue);
    }
  }

private:
  WeakPtr<const System> mSystem;
};

} // namespace OpenFDM

#endif
