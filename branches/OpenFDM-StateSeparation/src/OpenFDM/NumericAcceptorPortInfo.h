/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_NumericAcceptorPortInfo_H
#define OpenFDM_NumericAcceptorPortInfo_H

#include <string>
#include "AcceptorPortInfo.h"

namespace OpenFDM {

class NumericAcceptorPortInfo : public AcceptorPortInfo {
public:
  NumericAcceptorPortInfo(Node* node, const std::string& name, const Size& size) :
    AcceptorPortInfo(node, name),
    mSize(size)
  { }

private:
  Size mSize;
};

} // namespace OpenFDM

#endif
