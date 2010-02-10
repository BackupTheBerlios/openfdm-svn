/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ContinousStateInfoVector_H
#define OpenFDM_ContinousStateInfoVector_H

#include "ContinousStateInfo.h"
#include "TemplateInfoVector.h"

namespace OpenFDM {

class ContinousStateInfoVector : public TemplateInfoVector<ContinousStateInfo> {
};

} // namespace OpenFDM

#endif
