/* -*-c++-*- OpenFDM - Copyright (C) 2004-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NumericPortProxy_H
#define OpenFDM_NumericPortProxy_H

#include "NumericPortAcceptor.h"
#include "NumericPortProvider.h"

namespace OpenFDM {

class NumericPortProxy :
    public NumericPortAcceptor {
public:
  NumericPortProxy(Node* node, NumericPortProvider* provider) :
    NumericPortAcceptor(node),
    mPortProvider(provider)
  { }

  virtual ConnectResult setPortInterface(PortInterface* portInterface)
  {
    ConnectResult res = mPortProvider->setPortInterface(portInterface);
    if (!res)
      return res;
    return NumericPortAcceptor::setPortInterface(portInterface);
  }

  NumericPortProvider* getPortProvider()
  { return mPortProvider; }
  void setPortProvider(NumericPortProvider* provider)
  { mPortProvider = provider; }

private:
  SharedPtr<NumericPortProvider> mPortProvider;
};

} // namespace OpenFDM

#endif
