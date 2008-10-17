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

  virtual void apply(const PortInfo& portInfo, const PortValue* portValue)
  { }
  virtual void apply(const NumericPortInfo& portInfo,
                     const NumericPortValue* numericPortValue)
  { apply(static_cast<const PortInfo&>(portInfo), static_cast<const PortValue*>(numericPortValue)); }
  virtual void apply(const MechanicLinkInfo& portInfo,
                     const MechanicLinkValue* mechanicPortValue)
  { apply(static_cast<const PortInfo&>(portInfo), static_cast<const PortValue*>(mechanicPortValue)); }

  static SystemOutput* newDefaultSystemOutput(const std::string& filename);

protected:
  const AbstractNodeInstance* getNodeInstance(const NodePath& nodePath) const
  {
    SharedPtr<const System> system = mSystem.lock();
    if (!system)
      return 0;
    return system->getNodeInstance(nodePath);
  }
  virtual void apply(const NumericPortInfo& portInfo)
  {
    const AbstractNodeInstance* nodeInstance = getNodeInstance(getNodePath());
    if (!nodeInstance)
      return;
    apply(portInfo, nodeInstance->getPortValueList().getPortValue(portInfo));
  }
  virtual void apply(const MechanicLinkInfo& portInfo)
  {
    const AbstractNodeInstance* nodeInstance = getNodeInstance(getNodePath());
    if (!nodeInstance)
      return;
    apply(portInfo, nodeInstance->getPortValueList().getPortValue(portInfo));
  }
  virtual void apply(const PortInfo& portInfo)
  {
    const AbstractNodeInstance* nodeInstance = getNodeInstance(getNodePath());
    if (!nodeInstance)
      return;
    unsigned i = portInfo.getIndex();
    apply(portInfo, nodeInstance->getPortValueList().getPortValue(i));
  }

private:
  WeakPtr<const System> mSystem;
};

} // namespace OpenFDM

#endif
