/* -*-c++-*- OpenFDM - Copyright (C) 2007-2010 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_ContinousStateValueVector_H
#define OpenFDM_ContinousStateValueVector_H

#include "TemplateValueVector.h"
#include "ContinousStateInfo.h"
#include "ContinousStateValue.h"

namespace OpenFDM {

class ContinousStateValueVector :
    public TemplateValueVector<ContinousStateInfo, ContinousStateValue> {
public:
};

} // namespace OpenFDM

#endif
