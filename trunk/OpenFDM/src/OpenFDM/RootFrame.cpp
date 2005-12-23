/* -*-c++-*- OpenFDM - Copyright (C) 2004-2005 Mathias Froehlich 
 *
 */

#include "Frame.h"
#include "RootFrame.h"

namespace OpenFDM {

RootFrame::RootFrame(const std::string& name)
  : FreeFrame(name)
{
}

RootFrame::~RootFrame(void)
{
}

} // namespace OpenFDM
