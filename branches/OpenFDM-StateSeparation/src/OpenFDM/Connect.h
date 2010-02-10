/* -*-c++-*- OpenFDM - Copyright (C) 2004-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_Connect_H
#define OpenFDM_Connect_H

#include "Node.h"
#include "Port.h"
#include "SharedPtr.h"
#include "WeakPtr.h"

namespace OpenFDM {

class Group;

class Connect : public Referenced {
public:
  Connect(const Group* group);
  virtual ~Connect();

  SharedPtr<const Group> getGroup() const;

  SharedPtr<const Port> getPort0() const;
  bool setPort0(const Port* portInfo0);

  SharedPtr<const Port> getPort1() const;
  bool setPort1(const Port* portInfo1);

protected:
  bool isCompatible(const Port* portInfo0, const Port* portInfo1) const;
  bool isInGroup(const Port& portInfo) const;

private:
  WeakPtr<const Group> mGroup;
  WeakPtr<const Port> mPort0;
  WeakPtr<const Port> mPort1;
};

} // namespace OpenFDM

#endif
