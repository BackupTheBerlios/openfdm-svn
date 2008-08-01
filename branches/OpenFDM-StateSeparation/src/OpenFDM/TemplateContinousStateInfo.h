/* -*-c++-*- OpenFDM - Copyright (C) 2007-2008 Mathias Froehlich 
 *
 */

#ifndef OpenFDM_TemplateContinousStateInfo_H
#define OpenFDM_TemplateContinousStateInfo_H

#include "ContinousStateInfo.h"
#include "ContinousStateValue.h"
#include "TemplateStateInfo.h"

namespace OpenFDM {

template<typename T>
class TemplateContinousStateInfo : 
    public TemplateStateInfo<T, ContinousStateInfo, ContinousStateValue> {
};

} // namespace OpenFDM

#endif
