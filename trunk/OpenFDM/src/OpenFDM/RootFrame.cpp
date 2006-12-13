/* -*-c++-*- OpenFDM - Copyright (C) 2004-2006 Mathias Froehlich 
 *
 */

#include "RootFrame.h"

#include "Frame.h"

namespace OpenFDM {

RootFrame::RootFrame(const std::string& name)
  : FreeFrame(name)
{
}

RootFrame::~RootFrame(void)
{
}

} // namespace OpenFDM
