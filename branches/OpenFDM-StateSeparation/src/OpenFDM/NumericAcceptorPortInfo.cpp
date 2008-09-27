/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#include "NumericAcceptorPortInfo.h"

namespace OpenFDM {

NumericAcceptorPortInfo::NumericAcceptorPortInfo(Node* node,
        const std::string& name, const Size& size, bool directInput) :
  AcceptorPortInfo(node, name),
  mSize(size),
  mDirectInput(directInput)
{
}

NumericAcceptorPortInfo::~NumericAcceptorPortInfo()
{
}

} // namespace OpenFDM
