/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_MechanicAcceptorPortInfo_H
#define OpenFDM_MechanicAcceptorPortInfo_H

#include <string>
#include "AcceptorPortInfo.h"
#include "MechanicPortValue.h"

namespace OpenFDM {

class MechanicAcceptorPortInfo : public AcceptorPortInfo {
public:
  MechanicAcceptorPortInfo(Node* node, const std::string& name) :
    AcceptorPortInfo(node, name)
  { }

  virtual bool acceptPortValue(const PortValue* portValue) const
  { return dynamic_cast<const MechanicPortValue*>(portValue); }
};

} // namespace OpenFDM

#endif
