/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_PortInfo_H
#define OpenFDM_PortInfo_H

#include <string>
#include "Assert.h"
#include "PortValue.h"
#include "WeakPtr.h"
#include "WeakReferenced.h"

namespace OpenFDM {

// Some notes on ports:
// Naming:
// PortInfo - The meta data used to see what kind of port we have
// PortValue - The ports value during simulation
// PortHandle?? Hmmm??
// Port : NodeId, PortId ???

class Node;

class AcceptorPortInfo;
class ProviderPortInfo;
class ProxyAcceptorPortInfo;
class ProxyProviderPortInfo;

class PortInfo : public WeakReferenced {
public:
  PortInfo(Node* node, const std::string& name);
  virtual ~PortInfo();

  const std::string& getName() const { return mName; }
  void setName(const std::string& name);

  unsigned getIndex() const { return mIndex; }

  virtual AcceptorPortInfo* toAcceptorPortInfo()
  { return 0; }
  virtual const AcceptorPortInfo* toAcceptorPortInfo() const
  { return 0; }
  virtual ProviderPortInfo* toProviderPortInfo()
  { return 0; }
  virtual const ProviderPortInfo* toProviderPortInfo() const
  { return 0; }
  virtual ProxyAcceptorPortInfo* toProxyAcceptorPortInfo()
  { return 0; }
  virtual const ProxyAcceptorPortInfo* toProxyAcceptorPortInfo() const
  { return 0; }
  virtual ProxyProviderPortInfo* toProxyProviderPortInfo()
  { return 0; }
  virtual const ProxyProviderPortInfo* toProxyProviderPortInfo() const
  { return 0; }
  
  PortValue* getPortValue(const PortValueVector& portValueVector) const
  {
    unsigned index = getIndex();
    OpenFDMAssert(index < portValueVector.size());
    return portValueVector[index];
  }

  void clear();

private:
  PortInfo(const PortInfo&);
  PortInfo& operator=(const PortInfo&);

  void setIndex(unsigned index) { mIndex = index; }
  void invalidateIndex() { setIndex(~0u); }

  WeakPtr<Node> mNode;
  std::string mName;
  unsigned mIndex;

  // FIXME: Hmm, can I avoid this??
  friend class Node;
};

} // namespace OpenFDM

#endif
