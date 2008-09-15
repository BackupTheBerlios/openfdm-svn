/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_AcceptorPortInfo_H
#define OpenFDM_AcceptorPortInfo_H

#include <string>
#include "PortInfo.h"

namespace OpenFDM {

class AcceptorPortInfo : public PortInfo {
public:
  AcceptorPortInfo(Node* node, const std::string& name) :
    PortInfo(node, name)
  { }
  virtual ~AcceptorPortInfo() {}

  virtual AcceptorPortInfo* toAcceptorPortInfo()
  { return this; }
  virtual const AcceptorPortInfo* toAcceptorPortInfo() const
  { return this; }

  virtual bool getDirectInput() const
  { return false; }
};

} // namespace OpenFDM

#endif
