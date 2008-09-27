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
  AcceptorPortInfo(Node* node, const std::string& name);
  virtual ~AcceptorPortInfo();

  virtual const AcceptorPortInfo* toAcceptorPortInfo() const;

  virtual bool getDirectInput() const
  { return false; }
};

} // namespace OpenFDM

#endif
