/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Connect_H
#define OpenFDM_Connect_H

#include "Node.h"
#include "PortInfo.h"
#include "SharedPtr.h"
#include "WeakPtr.h"

namespace OpenFDM {

class Group;

class Connect : public Referenced {
public:
  Connect(const Group* group);
  virtual ~Connect();

  SharedPtr<const Group> getGroup() const;

  SharedPtr<const PortInfo> getPortInfo0() const;
  bool setPortInfo0(const PortInfo* portInfo0);

  SharedPtr<const PortInfo> getPortInfo1() const;
  bool setPortInfo1(const PortInfo* portInfo1);

protected:
  bool isCompatible(const PortInfo* portInfo0, const PortInfo* portInfo1) const;
  bool isInGroup(const PortInfo& portInfo) const;

private:
  WeakPtr<const Group> mGroup;
  WeakPtr<const PortInfo> mPortInfo0;
  WeakPtr<const PortInfo> mPortInfo1;
};

} // namespace OpenFDM

#endif
