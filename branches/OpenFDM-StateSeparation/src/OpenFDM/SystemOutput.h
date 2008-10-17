/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_SystemOutput_H
#define OpenFDM_SystemOutput_H

#include "System.h"
#include "ConstNodeVisitor.h"
#include "NodeInstance.h"

namespace OpenFDM {

class SystemOutput : public ConstNodeVisitor {
public:
  virtual ~SystemOutput();

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
                     const MechanicLinkValue* mechanicPortValue)
  { apply(portInfo, static_cast<const PortValue*>(mechanicPortValue)); }

  static SystemOutput* newDefaultSystemOutput(const std::string& filename);

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

      const MechanicLinkValue* mpv = portValue->toMechanicLinkValue();
      if (mpv) {
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
